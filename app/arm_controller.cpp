#include "arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <QColor>

#include "robotarm/inverse_kinematics.hpp"
#include "robotarm/payload_capacity.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

namespace {

// This arm (config/two_link_planar.json) has both joints rotating about Z
// with no link twist, so it lies entirely in a plane perpendicular to the
// default vertical gravity (rotating about the gravity axis never changes
// height). Using an in-plane gravity vector instead - same convention
// Module 3's tests and Module 5's training data already use - so the UI's
// torque numbers are actually meaningful, not trivially zero.
const Eigen::Vector3d kGravity(0.0, -9.81, 0.0);

QVector3D to_qvector3d(const Eigen::Vector3d &v) {
  return QVector3D(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

QColor torque_color(double torque, double limit) {
  const double ratio = limit > 0.0 ? std::abs(torque) / limit : 0.0;
  if (ratio < 0.6) return QColor(Qt::green);
  if (ratio < 0.9) return QColor(255, 191, 0); // amber
  return QColor(Qt::red);
}

QVariantList to_variant_list(const std::vector<double> &values) {
  QVariantList result;
  for (double value : values) result.append(value);
  return result;
}

QVariantList to_variant_list(const Eigen::VectorXd &values) {
  QVariantList result;
  for (Eigen::Index i = 0; i < values.size(); i++) result.append(values(i));
  return result;
}

} // namespace

ArmController::ArmController(const std::filesystem::path &config_path, const std::filesystem::path &surrogate_path,
                              const std::filesystem::path &dynamic_surrogate_path, QObject *parent)
    : QObject(parent), _config(load_arm_config(config_path)), _arm(to_arm_model(_config)),
      _links(to_link_masses(_config)), _inertialLinks(to_link_inertials(_config)),
      _jointAnglesRadians(_arm.num_joints(), 0.0), _jointVelocitiesRadians(_arm.num_joints(), 0.0),
      _jointAccelerationsRadians(_arm.num_joints(), 0.0) {
  try {
    _surrogate = std::make_unique<Surrogate>(surrogate_path);
  } catch (const std::exception &) {
    _surrogate = nullptr; // AI mode simply unavailable for this arm
  }
  if (!dynamic_surrogate_path.empty()) {
    try {
      _dynamicSurrogate = std::make_unique<Surrogate>(dynamic_surrogate_path);
    } catch (const std::exception &) {
      _dynamicSurrogate = nullptr;
    }
  }

  resetPayloadPosition();
  recompute();
}

QString ArmController::armName() const { return QString::fromStdString(_config.name); }

bool ArmController::aiModeAvailable() const {
  // Also requires the surrogate's own joint count to match the current
  // arm's - loadConfig() (Phase 4) can swap in an arm with a different
  // joint count than whatever the surrogate was trained on, in which case
  // predict()/predict_raw() would be fed the wrong number of inputs.
  const Surrogate *surrogate = _dynamicMode ? _dynamicSurrogate.get() : _surrogate.get();
  return surrogate != nullptr && surrogate->num_joints() == _arm.num_joints();
}

void ArmController::setPayloadKg(double kg) {
  if (kg == _payloadKg) return;
  _payloadKg = kg;
  recompute();
}

double ArmController::payloadX() const {
  // _jointPositions is the arm's own LOCAL frame (see the member's doc
  // comment) - the end effector's WORLD position while attached needs
  // the base offset added, same as jointPositions()/linkMidpoints().
  if (_payloadAttached && !_jointPositions.empty()) return _jointPositions.back().x() + _baseX;
  return _payloadX; // already world-space: placePayload() takes world meters directly
}

double ArmController::payloadY() const {
  if (_payloadAttached && !_jointPositions.empty()) return _jointPositions.back().y() + _baseY;
  return _payloadY;
}

void ArmController::placePayload(double xMeters, double yMeters) {
  if (_payloadAttached) return; // can't reposition what's currently held - open the gripper first
  _payloadX = xMeters;
  _payloadY = yMeters;
  emit resultsChanged();
}

void ArmController::resetPayloadPosition() {
  double total_reach = 0.0;
  for (const DHParameter &joint : _arm.joints()) total_reach += joint.a;
  _payloadX = total_reach * 0.45;
  _payloadY = -0.25;
  _payloadAttached = false;
}

void ArmController::setGripperOpen(bool open) {
  if (open == _gripperOpen) return;
  _gripperOpen = open;

  if (open) {
    // Opening: release the payload exactly where it currently is, if
    // it was being held - it doesn't snap back to some "home" spot.
    // _jointPositions is the LOCAL frame, so the drop spot needs the
    // base offset added to land at the end effector's actual WORLD
    // position (same reasoning as payloadX()/Y() above).
    if (_payloadAttached) {
      if (!_jointPositions.empty()) {
        _payloadX = _jointPositions.back().x() + _baseX;
        _payloadY = _jointPositions.back().y() + _baseY;
      }
      _payloadAttached = false;
    }
  } else if (!_payloadAttached && !_jointPositions.empty()) {
    // Closing: pick up the payload only if the end effector is
    // actually within reach of where it currently rests - closing the
    // gripper somewhere else in the plane just closes an empty hand.
    // _payloadX/Y are already world-space, so the end effector's
    // LOCAL position needs the same base offset added before the two
    // can be compared in the same frame.
    const double dx = (_jointPositions.back().x() + _baseX) - _payloadX;
    const double dy = (_jointPositions.back().y() + _baseY) - _payloadY;
    if (std::sqrt(dx * dx + dy * dy) <= kPickupRadiusMeters) {
      _payloadAttached = true;
    }
  }

  recompute();
}

void ArmController::setShowWorkspaceEnvelope(bool show) {
  if (show == _showWorkspaceEnvelope) return;
  _showWorkspaceEnvelope = show;
  emit resultsChanged(); // no physics affected - just the one changed property
}

void ArmController::setAiMode(bool enabled) {
  if (enabled == _aiMode || (enabled && !aiModeAvailable())) return;
  _aiMode = enabled;
  recompute();
}

void ArmController::setDynamicMode(bool enabled) {
  if (enabled == _dynamicMode) return;
  _dynamicMode = enabled;
  if (!enabled) _trajectoryScrubMode = false; // scrubbing implies dynamic mode
  if (_aiMode && !aiModeAvailable()) _aiMode = false;
  recompute();
}

void ArmController::setTrajectoryScrubMode(bool enabled) {
  if (enabled == _trajectoryScrubMode) return;
  _trajectoryScrubMode = enabled;
  if (enabled) {
    _dynamicMode = true;
    // Discards any previous recording - always starts fresh from
    // [rest, current pose] when scrubbing is (re-)enabled.
    const std::vector<double> rest(_jointAnglesRadians.size(), 0.0);
    _recordedTrajectory = std::make_unique<RecordedTrajectory>(
        std::vector<std::vector<double>>{rest, _jointAnglesRadians}, kTrajectoryDurationSeconds);
    _trajectoryProgress = 0.0;
    if (_aiMode && !aiModeAvailable()) _aiMode = false;
  }
  recompute();
}

void ArmController::setTrajectoryProgress(double progress) {
  const double clamped = std::clamp(progress, 0.0, 1.0);
  if (clamped == _trajectoryProgress) return;
  _trajectoryProgress = clamped;
  recompute();
}

int ArmController::trajectoryWaypointCount() const {
  return _recordedTrajectory ? static_cast<int>(_recordedTrajectory->num_waypoints()) : 0;
}

void ArmController::recordWaypoint() {
  if (!_trajectoryScrubMode || !_recordedTrajectory) return;
  _recordedTrajectory->add_waypoint(_jointAnglesRadians);
  recompute();
}

void ArmController::setJointAngleDegrees(int index, double degrees) {
  if (index < 0 || static_cast<size_t>(index) >= _jointAnglesRadians.size()) return;
  _jointAnglesRadians[static_cast<size_t>(index)] = to_radians(degrees);
  recompute();
}

void ArmController::moveToTarget(double xMeters, double yMeters) {
  // xMeters/yMeters arrive in WORLD space (QML always deals in world
  // coordinates - it's what's on screen) but solve_inverse_kinematics()
  // works in the arm's own LOCAL frame, so the base offset is
  // subtracted back out here, once, right at this boundary.
  const Eigen::Vector3d target(xMeters - _baseX, yMeters - _baseY, 0.0);
  _jointAnglesRadians = solve_inverse_kinematics(_arm, _jointAnglesRadians, target);
  recompute();
}

void ArmController::driveTo(double worldX, double worldY) {
  if (worldX == _baseX && worldY == _baseY) return;
  _baseX = worldX;
  _baseY = worldY;
  // No recompute(): the base offset doesn't feed into any torque or
  // angle calculation (see the header's doc comment) - every accessor
  // that depends on it (jointPositions(), linkMidpoints(), payloadX()/
  // Y()'s attached case, canvasCenter()) reads _baseX/_baseY live, so
  // just announcing the change is enough for QML to re-read them.
  emit resultsChanged();
}

void ArmController::driveNearPayload() {
  if (_payloadAttached) return; // already in hand - nowhere to "drive to"
  double total_reach = 0.0;
  for (const DHParameter &joint : _arm.joints()) total_reach += joint.a;
  // Parks half the arm's reach short of the payload, not directly on
  // top of it - see the Q_INVOKABLE doc comment for why.
  driveTo(_payloadX - total_reach * 0.5, _payloadY);
}

void ArmController::setJointVelocityDegrees(int index, double degreesPerSecond) {
  if (index < 0 || static_cast<size_t>(index) >= _jointVelocitiesRadians.size()) return;
  if (_trajectoryScrubMode) return; // the trajectory owns velocity while scrubbing
  _jointVelocitiesRadians[static_cast<size_t>(index)] = to_radians(degreesPerSecond);
  recompute();
}

void ArmController::setJointAccelerationDegrees(int index, double degreesPerSecondSquared) {
  if (index < 0 || static_cast<size_t>(index) >= _jointAccelerationsRadians.size()) return;
  if (_trajectoryScrubMode) return;
  _jointAccelerationsRadians[static_cast<size_t>(index)] = to_radians(degreesPerSecondSquared);
  recompute();
}

void ArmController::setJointTorqueLimit(int index, double limit) {
  if (index < 0 || static_cast<size_t>(index) >= _config.joints.size()) return;
  if (limit <= 0.0) return; // matches parse_arm_config()'s own validation
  _config.joints[static_cast<size_t>(index)].torque_limit = limit;
  emit resultsChanged();
}

void ArmController::setJointLength(int index, double meters) {
  if (index < 0 || static_cast<size_t>(index) >= _config.joints.size()) return;
  if (meters <= 0.0) return;

  JointConfig &joint = _config.joints[static_cast<size_t>(index)];
  joint.dh.a = meters;
  // Re-derive as a uniform rod of the new length - see the Q_INVOKABLE
  // doc comment in the header for why this always happens, even for a
  // joint that was loaded with an explicit inertia tensor.
  joint.inertial = LinkInertial::rod(joint.mass, meters);

  _arm = to_arm_model(_config);
  _links = to_link_masses(_config);
  _inertialLinks = to_link_inertials(_config);

  // The surrogates were trained on the arm's original geometry and
  // have no notion of link length as an input - their predictions
  // would silently stop matching the exact engine the moment this
  // changes, so AI mode isn't safe to leave on.
  _aiMode = false;

  // Any previous payload scan described the arm's OLD shape - stale
  // the moment its geometry changes, so clear it rather than leaving a
  // misleading curve on screen until the next scan overwrites it.
  _scanReachMeters.clear();
  _scanExactPayloadKg.clear();
  _scanAiPayloadKg.clear();

  recompute();
}

void ArmController::setJointMass(int index, double kg) {
  if (index < 0 || static_cast<size_t>(index) >= _config.joints.size()) return;
  if (kg < 0.0) return; // matches parse_arm_config()'s own rule - 0 itself is valid

  JointConfig &joint = _config.joints[static_cast<size_t>(index)];
  joint.mass = kg;
  joint.inertial = LinkInertial::rod(kg, joint.dh.a);

  // Geometry is untouched by a mass change - only the mass/inertia
  // data _links/_inertialLinks carry needs rebuilding, unlike
  // setJointLength()'s _arm = to_arm_model(_config).
  _links = to_link_masses(_config);
  _inertialLinks = to_link_inertials(_config);

  // Same reasoning as setJointLength(): the surrogate's training
  // data baked in the ORIGINAL masses, so its predictions would
  // silently stop matching reality the moment one changes.
  _aiMode = false;

  // Any previous payload scan described the arm's OLD weight - same
  // staleness reasoning as setJointLength().
  _scanReachMeters.clear();
  _scanExactPayloadKg.clear();
  _scanAiPayloadKg.clear();

  recompute();
}

void ArmController::addJoint() {
  JointConfig joint;
  joint.dh.type = JointType::Revolute;
  joint.dh.alpha = 0.0;
  joint.dh.d = 0.0;
  joint.dh.theta = 0.0;

  if (!_config.joints.empty()) {
    const JointConfig &last = _config.joints.back();
    joint.dh.a = std::max(0.1, last.dh.a * 0.7);
    joint.mass = std::max(0.1, last.mass * 0.6);
    joint.torque_limit = std::max(1.0, last.torque_limit * 0.5);
  } else {
    joint.dh.a = 0.5;
    joint.mass = 1.0;
    joint.torque_limit = 20.0;
  }
  joint.inertial = LinkInertial::rod(joint.mass, joint.dh.a);
  _config.joints.push_back(joint);

  _arm = to_arm_model(_config);
  _links = to_link_masses(_config);
  _inertialLinks = to_link_inertials(_config);

  _jointAnglesRadians.push_back(0.0);
  _jointVelocitiesRadians.push_back(0.0);
  _jointAccelerationsRadians.push_back(0.0);

  // A joint-count change invalidates everything loadConfig() already
  // invalidates for the same reason: the surrogate's joint count no
  // longer matches, any payload scan described the old shape, and a
  // trajectory recording's waypoints are sized to the OLD joint count.
  _aiMode = false;
  _scanReachMeters.clear();
  _scanExactPayloadKg.clear();
  _scanAiPayloadKg.clear();
  _trajectoryScrubMode = false;
  _recordedTrajectory.reset();

  recompute();
}

void ArmController::removeLastJoint() {
  if (_config.joints.size() <= 1) return; // an arm needs at least one joint

  _config.joints.pop_back();

  _arm = to_arm_model(_config);
  _links = to_link_masses(_config);
  _inertialLinks = to_link_inertials(_config);

  _jointAnglesRadians.pop_back();
  _jointVelocitiesRadians.pop_back();
  _jointAccelerationsRadians.pop_back();

  _aiMode = false;
  _scanReachMeters.clear();
  _scanExactPayloadKg.clear();
  _scanAiPayloadKg.clear();
  _trajectoryScrubMode = false;
  _recordedTrajectory.reset();

  recompute();
}

bool ArmController::loadConfig(const QString &path) {
  try {
    ArmConfig config = load_arm_config(path.toStdString());
    _config = std::move(config);
    _arm = to_arm_model(_config);
    _links = to_link_masses(_config);
    _inertialLinks = to_link_inertials(_config);

    // None of the current pose/payload/mode state necessarily still makes
    // sense for a different arm - most obviously, a different joint
    // count - so reset to defaults rather than trying to carry it over.
    const size_t n = _arm.num_joints();
    _jointAnglesRadians.assign(n, 0.0);
    _jointVelocitiesRadians.assign(n, 0.0);
    _jointAccelerationsRadians.assign(n, 0.0);
    _payloadKg = 0.0;
    _gripperOpen = true;
    resetPayloadPosition();
    _aiMode = false;
    _dynamicMode = false;
    _trajectoryScrubMode = false;
    _trajectoryProgress = 0.0;
    _recordedTrajectory.reset();

    // A different arm invalidates any previous payload scan the same
    // way setJointLength() does - see that function's own comment.
    _scanReachMeters.clear();
    _scanExactPayloadKg.clear();
    _scanAiPayloadKg.clear();

    _configMessage = QString("Loaded '%1' (%2 joints)").arg(armName()).arg(n);
    recompute();
    return true;
  } catch (const std::exception &e) {
    _configMessage = QString("Load failed: %1").arg(e.what());
    emit resultsChanged();
    return false;
  }
}

bool ArmController::saveConfig(const QString &path) {
  try {
    save_arm_config(_config, path.toStdString());
    _configMessage = QString("Saved to %1").arg(path);
    emit resultsChanged();
    return true;
  } catch (const std::exception &e) {
    _configMessage = QString("Save failed: %1").arg(e.what());
    emit resultsChanged();
    return false;
  }
}

void ArmController::runPayloadScan(int sampleCount) {
  const int n = std::clamp(sampleCount, 2, 300);
  const size_t joint_count = _arm.num_joints();
  if (joint_count == 0) return;

  std::vector<double> limits;
  limits.reserve(_config.joints.size());
  for (const JointConfig &joint : _config.joints) limits.push_back(joint.torque_limit);

  // Sweep from fully extended (t=0) to folded (t=1): the shoulder
  // stays at whatever angle it's currently at, while every other
  // joint bends together by up to 110 degrees, continuously shrinking
  // the end effector's distance from the base - the "reach" axis of
  // the derating curve this is meant to produce.
  const double shoulder_angle = _jointAnglesRadians[0];
  const double fold_max_radians = to_radians(110.0);
  const std::vector<double> zero_rates(joint_count, 0.0);
  const bool dynamic = _dynamicMode;

  std::vector<std::vector<double>> poses(static_cast<size_t>(n));
  std::vector<double> reach_meters(static_cast<size_t>(n));
  for (int i = 0; i < n; i++) {
    const double t = (n > 1) ? static_cast<double>(i) / static_cast<double>(n - 1) : 0.0;
    std::vector<double> pose(joint_count, 0.0);
    pose[0] = shoulder_angle;
    for (size_t j = 1; j < joint_count; j++) pose[j] = -t * fold_max_radians;
    poses[static_cast<size_t>(i)] = pose;

    const std::vector<Eigen::Matrix4d> frames = _arm.forward_kinematics(pose);
    Eigen::Vector3d tip = Eigen::Vector3d::Zero();
    if (!frames.empty()) tip = frames.back().block<3, 1>(0, 3);
    reach_meters[static_cast<size_t>(i)] = tip.norm();
  }

  std::vector<double> exact_payload_kg(static_cast<size_t>(n));
  const auto exact_start = std::chrono::steady_clock::now();
  for (int i = 0; i < n; i++) {
    const std::vector<double> &pose = poses[static_cast<size_t>(i)];
    auto evaluator = [&](double candidate_payload) -> Eigen::VectorXd {
      if (dynamic) return inverse_dynamics(_arm, pose, zero_rates, zero_rates, _inertialLinks, candidate_payload, kGravity);
      return static_joint_torques(_arm, pose, _links, candidate_payload, kGravity);
    };
    exact_payload_kg[static_cast<size_t>(i)] = max_safe_payload(evaluator, limits, kPayloadSearchCeilingKg);
  }
  const auto exact_end = std::chrono::steady_clock::now();
  _scanExactMilliseconds = std::chrono::duration<double, std::milli>(exact_end - exact_start).count();

  _scanAiAvailable = aiModeAvailable();
  std::vector<double> ai_payload_kg;
  if (_scanAiAvailable) {
    const Surrogate *surrogate = dynamic ? _dynamicSurrogate.get() : _surrogate.get();
    ai_payload_kg.resize(static_cast<size_t>(n));
    const auto ai_start = std::chrono::steady_clock::now();
    for (int i = 0; i < n; i++) {
      const std::vector<double> &pose = poses[static_cast<size_t>(i)];
      auto evaluator = [&](double candidate_payload) -> Eigen::VectorXd {
        std::vector<double> raw_input = pose;
        if (dynamic) {
          raw_input.insert(raw_input.end(), zero_rates.begin(), zero_rates.end());
          raw_input.insert(raw_input.end(), zero_rates.begin(), zero_rates.end());
        }
        raw_input.push_back(candidate_payload);
        const std::vector<double> result = surrogate->predict_raw(raw_input);
        Eigen::VectorXd tau(static_cast<Eigen::Index>(result.size()));
        for (size_t k = 0; k < result.size(); k++) tau(static_cast<Eigen::Index>(k)) = result[k];
        return tau;
      };
      ai_payload_kg[static_cast<size_t>(i)] = max_safe_payload(evaluator, limits, kPayloadSearchCeilingKg);
    }
    const auto ai_end = std::chrono::steady_clock::now();
    _scanAiMilliseconds = std::chrono::duration<double, std::milli>(ai_end - ai_start).count();
  } else {
    _scanAiMilliseconds = 0.0;
  }

  _scanReachMeters = reach_meters;
  _scanExactPayloadKg = exact_payload_kg;
  _scanAiPayloadKg = ai_payload_kg;

  emit resultsChanged();
}

TrajectoryState ArmController::effectiveState() const {
  if (_trajectoryScrubMode && _recordedTrajectory) {
    return _recordedTrajectory->evaluate(_trajectoryProgress * _recordedTrajectory->total_duration());
  }

  TrajectoryState state;
  state.position = _jointAnglesRadians;
  const size_t n = _arm.num_joints();
  state.velocity = _dynamicMode ? _jointVelocitiesRadians : std::vector<double>(n, 0.0);
  state.acceleration = _dynamicMode ? _jointAccelerationsRadians : std::vector<double>(n, 0.0);
  return state;
}

void ArmController::recompute() {
  const TrajectoryState state = effectiveState();
  const std::vector<double> &positions = state.position;
  const std::vector<double> &velocities = state.velocity;
  const std::vector<double> &accelerations = state.acceleration;
  const double payload = effectivePayloadKg();

  if (_dynamicMode) {
    if (_aiMode && _dynamicSurrogate) {
      std::vector<double> raw_input = positions;
      raw_input.insert(raw_input.end(), velocities.begin(), velocities.end());
      raw_input.insert(raw_input.end(), accelerations.begin(), accelerations.end());
      raw_input.push_back(payload);
      _jointTorques = _dynamicSurrogate->predict_raw(raw_input);
    } else {
      const Eigen::VectorXd tau =
          inverse_dynamics(_arm, positions, velocities, accelerations, _inertialLinks, payload, kGravity);
      _jointTorques.assign(static_cast<size_t>(tau.size()), 0.0);
      for (Eigen::Index i = 0; i < tau.size(); i++) _jointTorques[static_cast<size_t>(i)] = tau(i);
    }
  } else {
    if (_aiMode && _surrogate) {
      _jointTorques = _surrogate->predict(positions, payload);
    } else {
      const Eigen::VectorXd tau = static_joint_torques(_arm, positions, _links, payload, kGravity);
      _jointTorques.assign(static_cast<size_t>(tau.size()), 0.0);
      for (Eigen::Index i = 0; i < tau.size(); i++) _jointTorques[static_cast<size_t>(i)] = tau(i);
    }
  }

  // Always computed via the exact engine - the AI surrogates predict a
  // combined torque only, not a decomposition.
  _breakdown = decompose_torque(_arm, positions, velocities, accelerations, _inertialLinks, payload, kGravity);

  const std::vector<Eigen::Matrix4d> frames = _arm.forward_kinematics(positions);
  _jointPositions.clear();
  _jointPositions.reserve(frames.size() + 1);
  _jointPositions.push_back(Eigen::Vector3d::Zero()); // base origin
  for (const Eigen::Matrix4d &frame : frames) _jointPositions.push_back(frame.block<3, 1>(0, 3));

  // "How much more could this arm safely lift right now" - always via
  // the exact engine (see the Q_PROPERTY doc comment), at the current
  // pose/velocity/acceleration so it's asking about exactly what's on
  // screen, not some other state.
  std::vector<double> limits;
  limits.reserve(_config.joints.size());
  for (const JointConfig &joint : _config.joints) limits.push_back(joint.torque_limit);
  auto exact_evaluator = [&](double candidate_payload) -> Eigen::VectorXd {
    if (_dynamicMode) {
      return inverse_dynamics(_arm, positions, velocities, accelerations, _inertialLinks, candidate_payload,
                               kGravity);
    }
    return static_joint_torques(_arm, positions, _links, candidate_payload, kGravity);
  };
  _currentMaxSafePayloadKg = max_safe_payload(exact_evaluator, limits, kPayloadSearchCeilingKg);

  emit resultsChanged();
}

QVariantList ArmController::jointAnglesDegrees() const {
  QVariantList result;
  for (double radians : effectiveState().position) result.append(to_degrees(radians));
  return result;
}

QVariantList ArmController::jointVelocitiesDegrees() const {
  QVariantList result;
  for (double radians : effectiveState().velocity) result.append(to_degrees(radians));
  return result;
}

QVariantList ArmController::jointAccelerationsDegrees() const {
  QVariantList result;
  for (double radians : effectiveState().acceleration) result.append(to_degrees(radians));
  return result;
}

QVariantList ArmController::jointTorques() const { return to_variant_list(_jointTorques); }

QVariantList ArmController::jointTorqueLimits() const {
  QVariantList result;
  for (const JointConfig &joint : _config.joints) result.append(joint.torque_limit);
  return result;
}

QVariantList ArmController::jointMassesKg() const {
  QVariantList result;
  for (const JointConfig &joint : _config.joints) result.append(joint.mass);
  return result;
}

QVariantList ArmController::jointColors() const {
  QVariantList result;
  for (size_t i = 0; i < _jointTorques.size() && i < _config.joints.size(); i++) {
    result.append(torque_color(_jointTorques[i], _config.joints[i].torque_limit));
  }
  return result;
}

QVariantList ArmController::gravityTorques() const { return to_variant_list(_breakdown.gravity); }
QVariantList ArmController::inertiaTorques() const { return to_variant_list(_breakdown.inertia); }
QVariantList ArmController::coriolisTorques() const { return to_variant_list(_breakdown.coriolis); }

// Pivot the camera around the middle of the arm's reach, offset by the
// base's own world position - not the world origin (0,0,0), per the
// roadmap's prior-art finding that orbiting around world-origin is
// frustrating once the arm extends away from it, and not just the
// arm's own local-frame midpoint either, now that driveTo() can move
// that frame anywhere. Computed fresh each call (not cached) since it
// depends on _baseX/_baseY as well as the arm's own geometry, and both
// can change independently of each other.
QVector3D ArmController::canvasCenter() const {
  double total_reach = 0.0;
  for (const DHParameter &joint : _arm.joints()) total_reach += joint.a;
  return QVector3D(static_cast<float>(_baseX + total_reach / 2.0), static_cast<float>(_baseY), 0.0f);
}

QVariantList ArmController::jointPositions() const {
  const Eigen::Vector3d base_offset(_baseX, _baseY, 0.0);
  QVariantList result;
  for (const Eigen::Vector3d &position : _jointPositions) result.append(to_qvector3d(position + base_offset));
  return result;
}

QVariantList ArmController::linkMidpoints() const {
  const Eigen::Vector3d base_offset(_baseX, _baseY, 0.0);
  QVariantList result;
  for (size_t i = 0; i + 1 < _jointPositions.size(); i++) {
    result.append(to_qvector3d((_jointPositions[i] + _jointPositions[i + 1]) / 2.0 + base_offset));
  }
  return result;
}

QVariantList ArmController::linkLengths() const {
  QVariantList result;
  for (size_t i = 0; i + 1 < _jointPositions.size(); i++) {
    result.append((_jointPositions[i + 1] - _jointPositions[i]).norm());
  }
  return result;
}

QVariantList ArmController::scanReachMeters() const { return to_variant_list(_scanReachMeters); }
QVariantList ArmController::scanExactPayloadKg() const { return to_variant_list(_scanExactPayloadKg); }
QVariantList ArmController::scanAiPayloadKg() const { return to_variant_list(_scanAiPayloadKg); }

QVariantList ArmController::linkRotations() const {
  QVariantList result;
  for (size_t i = 0; i + 1 < _jointPositions.size(); i++) {
    const Eigen::Vector3d direction_eigen = _jointPositions[i + 1] - _jointPositions[i];
    const double length = direction_eigen.norm();
    if (length < 1e-9) {
      result.append(QQuaternion());
      continue;
    }
    const QVector3D direction = to_qvector3d(direction_eigen / length);
    // Built-in cylinder/cone primitives run along local Y by default - the
    // shortest-arc rotation from Y to the link's direction orients them.
    result.append(QQuaternion::rotationTo(QVector3D(0.0f, 1.0f, 0.0f), direction));
  }
  return result;
}

} // namespace robotarm
