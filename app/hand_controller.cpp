#include "hand_controller.hpp"

#include <algorithm>

#include <QDebug>

#include "robotarm/units.hpp"

namespace robotarm {

namespace {
QVector3D to_qvector3d(const Eigen::Vector3d &v) {
  return QVector3D(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

QQuaternion to_qquaternion(const Eigen::Matrix3d &rotation) {
  const Eigen::Quaterniond q(rotation);
  return QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
                     static_cast<float>(q.z()));
}

// Rotation whose local +Y lies along `along` and local +X along `hinge`
// (re-orthogonalized against `along`), with +Z = X x Y. Falls back to an
// arbitrary perpendicular if the hinge is parallel to the link.
QQuaternion link_frame(const Eigen::Vector3d &along, const Eigen::Vector3d &hinge) {
  const double length = along.norm();
  if (length < 1e-9) {
    return QQuaternion();
  }
  const Eigen::Vector3d y = along / length;
  Eigen::Vector3d x = hinge - hinge.dot(y) * y;
  if (x.norm() < 1e-9) {
    x = y.unitOrthogonal();
  }
  x.normalize();
  const Eigen::Vector3d z = x.cross(y);
  Eigen::Matrix3d rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;
  return to_qquaternion(rotation);
}
} // namespace

HandController::HandController(const std::filesystem::path &hand_config_path, QObject *parent)
    : QObject(parent), _config(load_hand_config(hand_config_path)), _tree(to_kinematic_tree(_config)),
      _gestures(_config) {
  for (const KinematicChain &chain : _tree.chains()) _chainOrder.push_back(chain.name);

  _jointAnglesRadians = _gestures.restPose();
  recompute();

  _animationTicker.setInterval(kAnimationTickMilliseconds);
  connect(&_animationTicker, &QTimer::timeout, this, &HandController::advanceAnimation);
}

QString HandController::handName() const {
  return QString::fromStdString(_config.name);
}

QStringList HandController::chainNames() const {
  QStringList names;
  for (const std::string &name : _chainOrder) names.append(QString::fromStdString(name));
  return names;
}

const HandChainConfig *HandController::findChainConfig(const std::string &name) const {
  for (const HandChainConfig &chain : _config.chains) {
    if (chain.name == name) return &chain;
  }
  return nullptr;
}

int HandController::jointCountForChain(const QString &chainName) const {
  const HandChainConfig *chain = findChainConfig(chainName.toStdString());
  return chain == nullptr ? 0 : static_cast<int>(chain->joints.size());
}

double HandController::jointAngleDegrees(const QString &chainName, int jointIndex) const {
  const auto it = _jointAnglesRadians.find(chainName.toStdString());
  if (it == _jointAnglesRadians.end()) return 0.0;
  if (jointIndex < 0 || static_cast<size_t>(jointIndex) >= it->second.size()) return 0.0;
  return to_degrees(it->second[static_cast<size_t>(jointIndex)]);
}

double HandController::jointMinDegrees(const QString &chainName, int jointIndex) const {
  const HandChainConfig *chain = findChainConfig(chainName.toStdString());
  if (chain == nullptr || jointIndex < 0 || static_cast<size_t>(jointIndex) >= chain->joints.size()) return 0.0;
  return chain->joints[static_cast<size_t>(jointIndex)].min_degrees;
}

double HandController::jointMaxDegrees(const QString &chainName, int jointIndex) const {
  const HandChainConfig *chain = findChainConfig(chainName.toStdString());
  if (chain == nullptr || jointIndex < 0 || static_cast<size_t>(jointIndex) >= chain->joints.size()) return 0.0;
  return chain->joints[static_cast<size_t>(jointIndex)].max_degrees;
}

void HandController::setJointAngleDegrees(const QString &chainName, int jointIndex, double degrees) {
  const std::string name = chainName.toStdString();
  const HandChainConfig *chain = findChainConfig(name);
  if (chain == nullptr || jointIndex < 0 || static_cast<size_t>(jointIndex) >= chain->joints.size()) {
    qWarning() << "HandController: setJointAngleDegrees ignored, unknown chain/joint" << chainName << jointIndex;
    return;
  }

  const HandJointConfig &joint = chain->joints[static_cast<size_t>(jointIndex)];
  if (joint.min_degrees == joint.max_degrees) return; // fixed placement joint - never moves

  const double clamped = std::clamp(degrees, joint.min_degrees, joint.max_degrees);
  _jointAnglesRadians[name][static_cast<size_t>(jointIndex)] = to_radians(clamped);

  // A manual, direct edit interrupts whatever gesture animation was
  // playing - the two shouldn't fight over the same joints.
  _animation.reset();
  _animationTicker.stop();
  _currentGesture = QStringLiteral("manual");

  recompute();
  emit resultsChanged();
}

std::vector<Eigen::Vector3d> HandController::chainPositions(const std::string &name) const {
  std::vector<Eigen::Vector3d> positions;
  const auto it = _jointPoses.find(name);
  if (it == _jointPoses.end()) return positions;
  positions.reserve(it->second.size());
  for (const Eigen::Matrix4d &pose : it->second) positions.push_back(pose.block<3, 1>(0, 3));
  return positions;
}

QVariantList HandController::jointPositionsForChain(const QString &chainName) const {
  QVariantList result;
  for (const Eigen::Vector3d &p : chainPositions(chainName.toStdString())) result.append(to_qvector3d(p));
  return result;
}

QVariantList HandController::linkMidpointsForChain(const QString &chainName) const {
  QVariantList result;
  const std::vector<Eigen::Vector3d> positions = chainPositions(chainName.toStdString());
  for (size_t i = 0; i + 1 < positions.size(); i++) {
    result.append(to_qvector3d((positions[i] + positions[i + 1]) / 2.0));
  }
  return result;
}

QVariantList HandController::linkLengthsForChain(const QString &chainName) const {
  QVariantList result;
  const std::vector<Eigen::Vector3d> positions = chainPositions(chainName.toStdString());
  for (size_t i = 0; i + 1 < positions.size(); i++) result.append((positions[i + 1] - positions[i]).norm());
  return result;
}

QVariantList HandController::linkRotationsForChain(const QString &chainName) const {
  QVariantList result;
  const std::vector<Eigen::Vector3d> positions = chainPositions(chainName.toStdString());
  for (size_t i = 0; i + 1 < positions.size(); i++) {
    const Eigen::Vector3d direction_eigen = positions[i + 1] - positions[i];
    const double length = direction_eigen.norm();
    if (length < 1e-9) {
      result.append(QQuaternion());
      continue;
    }
    const QVector3D direction = to_qvector3d(direction_eigen / length);
    result.append(QQuaternion::rotationTo(QVector3D(0.0f, 1.0f, 0.0f), direction));
  }
  return result;
}

HandController::RootSegment HandController::rootSegmentFor(const std::string &name) const {
  RootSegment seg;
  const HandChainConfig *chain = findChainConfig(name);
  if (chain == nullptr || chain->parent_chain.empty() || chain->joints.empty()) return seg;

  const std::vector<Eigen::Vector3d> my_positions = chainPositions(name);
  if (my_positions.empty()) return seg;

  const Eigen::Matrix4d base = _tree.chain_base_frame(name, _jointPoses);
  const Eigen::Vector3d parent_tip = base.block<3, 1>(0, 3); // the chain's own base, origin included
  const Eigen::Vector3d &my_first = my_positions.front();
  seg.frame = link_frame(my_first - parent_tip, base.block<3, 1>(0, 2));

  seg.present = true;
  seg.midpoint = (parent_tip + my_first) / 2.0;
  const Eigen::Vector3d direction_eigen = my_first - parent_tip;
  seg.length = direction_eigen.norm();
  if (seg.length < 1e-9) {
    seg.rotation = QQuaternion();
  } else {
    const QVector3D direction = to_qvector3d(direction_eigen / seg.length);
    seg.rotation = QQuaternion::rotationTo(QVector3D(0.0f, 1.0f, 0.0f), direction);
  }
  return seg;
}

QVariantList HandController::rootSegmentMidpointForChain(const QString &chainName) const {
  const RootSegment seg = rootSegmentFor(chainName.toStdString());
  if (!seg.present) return {};
  return {QVariant::fromValue(to_qvector3d(seg.midpoint))};
}

QVariantList HandController::rootSegmentLengthForChain(const QString &chainName) const {
  const RootSegment seg = rootSegmentFor(chainName.toStdString());
  if (!seg.present) return {};
  return {QVariant::fromValue(seg.length)};
}

QVariantList HandController::rootSegmentRotationForChain(const QString &chainName) const {
  const RootSegment seg = rootSegmentFor(chainName.toStdString());
  if (!seg.present) return {};
  return {QVariant::fromValue(seg.rotation)};
}

QVariantList HandController::rootSegmentFrameForChain(const QString &chainName) const {
  const RootSegment seg = rootSegmentFor(chainName.toStdString());
  if (!seg.present) {
    return {};
  }
  return {QVariant::fromValue(seg.frame)};
}

QVariantList HandController::linkFramesForChain(const QString &chainName) const {
  QVariantList result;
  const auto it = _jointPoses.find(chainName.toStdString());
  if (it == _jointPoses.end()) {
    return result;
  }
  const std::vector<Eigen::Matrix4d> &poses = it->second;
  for (size_t i = 0; i + 1 < poses.size(); i++) {
    // Link i runs from frame i's origin to frame i+1's origin and is swung
    // by joint i+1, whose axis is frame i's z.
    const Eigen::Vector3d along = poses[i + 1].block<3, 1>(0, 3) - poses[i].block<3, 1>(0, 3);
    result.append(QVariant::fromValue(link_frame(along, poses[i].block<3, 1>(0, 2))));
  }
  return result;
}

QVector3D HandController::chainBasePosition(const QString &chainName) const {
  const std::string name = chainName.toStdString();
  if (findChainConfig(name) == nullptr) {
    return QVector3D();
  }
  return to_qvector3d(_tree.chain_base_frame(name, _jointPoses).block<3, 1>(0, 3));
}

Eigen::Matrix4d HandController::rootTipFrame() const {
  for (const HandChainConfig &chain : _config.chains) {
    if (!chain.parent_chain.empty()) {
      continue;
    }
    const auto it = _jointPoses.find(chain.name);
    if (it != _jointPoses.end() && !it->second.empty()) {
      return it->second.back();
    }
    return _tree.chain_base_frame(chain.name, _jointPoses);
  }
  return Eigen::Matrix4d::Identity();
}

QVector3D HandController::palmPosition() const { return to_qvector3d(rootTipFrame().block<3, 1>(0, 3)); }

QQuaternion HandController::palmRotation() const {
  return to_qquaternion(rootTipFrame().block<3, 3>(0, 0));
}

void HandController::recompute() {
  _jointPoses = _tree.forward_kinematics(_jointAnglesRadians);
  _revision++;
}

std::vector<double> HandController::flatten(const std::map<std::string, std::vector<double>> &pose_by_chain) const {
  std::vector<double> flat;
  for (const std::string &name : _chainOrder) {
    const std::vector<double> &angles = pose_by_chain.at(name);
    flat.insert(flat.end(), angles.begin(), angles.end());
  }
  return flat;
}

std::map<std::string, std::vector<double>> HandController::unflatten(const std::vector<double> &flat) const {
  std::map<std::string, std::vector<double>> pose;
  size_t offset = 0;
  for (const std::string &name : _chainOrder) {
    const size_t count = _tree.chain(name).joints.size();
    pose[name] = std::vector<double>(flat.begin() + static_cast<long>(offset),
                                      flat.begin() + static_cast<long>(offset + count));
    offset += count;
  }
  return pose;
}

void HandController::startAnimationTo(const std::vector<std::map<std::string, std::vector<double>>> &target_waypoints) {
  std::vector<std::vector<double>> flattened;
  flattened.reserve(target_waypoints.size() + 1);
  flattened.push_back(flatten(_jointAnglesRadians));
  for (const auto &waypoint : target_waypoints) flattened.push_back(flatten(waypoint));

  _animation = std::make_unique<RecordedTrajectory>(std::move(flattened), kGestureSegmentSeconds);
  _animationClock.start();
  _animationTicker.start();
}

void HandController::playGesture(const QString &name) {
  if (name == QStringLiteral("restPose")) {
    startAnimationTo({_gestures.restPose()});
  } else if (name == QStringLiteral("openPalm")) {
    startAnimationTo({_gestures.openPalm()});
  } else if (name == QStringLiteral("closedFist")) {
    startAnimationTo({_gestures.closedFist()});
  } else if (name == QStringLiteral("powerGrip")) {
    startAnimationTo({_gestures.powerGrip()});
  } else if (name == QStringLiteral("wave")) {
    startAnimationTo(_gestures.wave());
  } else {
    qWarning() << "HandController: unknown gesture" << name;
    return;
  }

  _currentGesture = name;
  emit resultsChanged();
}

void HandController::advanceAnimation() {
  if (!_animation) {
    _animationTicker.stop();
    return;
  }

  const double t = static_cast<double>(_animationClock.elapsed()) / 1000.0;
  const TrajectoryState state = _animation->evaluate(t);
  _jointAnglesRadians = unflatten(state.position);
  recompute();

  if (t >= _animation->total_duration()) {
    _animation.reset();
    _animationTicker.stop();
  }

  emit resultsChanged();
}

} // namespace robotarm
