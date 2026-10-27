#include "arm_controller.hpp"

#include <algorithm>
#include <cmath>

#include <QColor>

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

void ArmController::setGripperOpen(bool open) {
  if (open == _gripperOpen) return;
  _gripperOpen = open;
  recompute();
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
    _gripperOpen = false;
    _aiMode = false;
    _dynamicMode = false;
    _trajectoryScrubMode = false;
    _trajectoryProgress = 0.0;
    _recordedTrajectory.reset();

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

  // Pivot the camera around the middle of the arm's reach, not the world
  // origin (0,0,0) - the roadmap's prior-art finding that orbiting around
  // world-origin is frustrating once the arm extends away from it.
  // Recomputed here (not just once at construction) so it stays correct
  // after loadConfig() or setJointLength() changes the arm's geometry.
  double total_reach = 0.0;
  for (const DHParameter &joint : _arm.joints()) total_reach += joint.a;
  _canvasCenter = QVector3D(static_cast<float>(total_reach / 2.0), 0.0f, 0.0f);

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

QVariantList ArmController::jointPositions() const {
  QVariantList result;
  for (const Eigen::Vector3d &position : _jointPositions) result.append(to_qvector3d(position));
  return result;
}

QVariantList ArmController::linkMidpoints() const {
  QVariantList result;
  for (size_t i = 0; i + 1 < _jointPositions.size(); i++) {
    result.append(to_qvector3d((_jointPositions[i] + _jointPositions[i + 1]) / 2.0));
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
