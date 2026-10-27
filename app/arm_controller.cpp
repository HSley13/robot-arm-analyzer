#include "arm_controller.hpp"

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

} // namespace

ArmController::ArmController(const std::filesystem::path &config_path, const std::filesystem::path &surrogate_path,
                              QObject *parent)
    : QObject(parent), _config(load_arm_config(config_path)), _arm(to_arm_model(_config)),
      _links(to_link_masses(_config)), _jointAnglesRadians(_arm.num_joints(), 0.0) {
  try {
    _surrogate = std::make_unique<Surrogate>(surrogate_path);
  } catch (const std::exception &) {
    _surrogate = nullptr; // AI mode simply unavailable for this arm
  }

  double total_reach = 0.0;
  for (const DHParameter &joint : _arm.joints()) total_reach += joint.a;
  // Pivot the camera around the middle of the arm's reach, not the world
  // origin (0,0,0) - the roadmap's prior-art finding that orbiting around
  // world-origin is frustrating once the arm extends away from it.
  _canvasCenter = QVector3D(static_cast<float>(total_reach / 2.0), 0.0f, 0.0f);

  recompute();
}

QString ArmController::armName() const { return QString::fromStdString(_config.name); }

void ArmController::setPayloadKg(double kg) {
  if (kg == _payloadKg) return;
  _payloadKg = kg;
  recompute();
}

void ArmController::setAiMode(bool enabled) {
  if (enabled == _aiMode || (enabled && !aiModeAvailable())) return;
  _aiMode = enabled;
  recompute();
}

void ArmController::setJointAngleDegrees(int index, double degrees) {
  if (index < 0 || static_cast<size_t>(index) >= _jointAnglesRadians.size()) return;
  _jointAnglesRadians[static_cast<size_t>(index)] = to_radians(degrees);
  recompute();
}

void ArmController::recompute() {
  if (_aiMode && _surrogate) {
    _jointTorques = _surrogate->predict(_jointAnglesRadians, _payloadKg);
  } else {
    const Eigen::VectorXd tau = static_joint_torques(_arm, _jointAnglesRadians, _links, _payloadKg, kGravity);
    _jointTorques.assign(static_cast<size_t>(tau.size()), 0.0);
    for (Eigen::Index i = 0; i < tau.size(); i++) _jointTorques[static_cast<size_t>(i)] = tau(i);
  }

  const std::vector<Eigen::Matrix4d> frames = _arm.forward_kinematics(_jointAnglesRadians);
  _jointPositions.clear();
  _jointPositions.reserve(frames.size() + 1);
  _jointPositions.push_back(Eigen::Vector3d::Zero()); // base origin
  for (const Eigen::Matrix4d &frame : frames) _jointPositions.push_back(frame.block<3, 1>(0, 3));

  emit resultsChanged();
}

QVariantList ArmController::jointAnglesDegrees() const {
  QVariantList result;
  for (double radians : _jointAnglesRadians) result.append(to_degrees(radians));
  return result;
}

QVariantList ArmController::jointTorques() const {
  QVariantList result;
  for (double torque : _jointTorques) result.append(torque);
  return result;
}

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
