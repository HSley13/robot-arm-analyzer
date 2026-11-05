#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <QElapsedTimer>
#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>
#include <QVector3D>

#include "robotarm/hand_config.hpp"
#include "robotarm/hand_gestures.hpp"
#include "robotarm/kinematic_tree.hpp"
#include "robotarm/recorded_trajectory.hpp"

namespace robotarm {

// QML-facing backend for a hand, built the same way ArmController bridges
// ArmModel/statics to QML: only plain Qt types cross the boundary, never
// an Eigen matrix or a core/ struct directly.
//
// Deliberately does NOT hold a pointer to ArmController. The hand's own
// forward kinematics is entirely in the hand's own local frame (its root
// chain's base); placing that frame correctly at the end of the arm is a
// QML scene-graph concern (the hand's root Node is parented under the
// arm's end-effector Node), not something this class needs to know about
// to do its own job. That keeps the two controllers decoupled - nothing
// here needs to change if the arm's own pose representation ever does.
class HandController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString handName READ handName CONSTANT)
  Q_PROPERTY(QStringList chainNames READ chainNames CONSTANT)
  Q_PROPERTY(QString currentGesture READ currentGesture NOTIFY resultsChanged)
  Q_PROPERTY(bool animating READ animating NOTIFY resultsChanged)
  // Increments on every recompute() - exists purely so a QML binding that
  // calls a per-chain Q_INVOKABLE getter (jointPositionsForChain() etc.,
  // which can't be a real Q_PROPERTY since they take an argument) has a
  // real property to depend on. Without reading *something* with a
  // NOTIFY tied to resultsChanged, QML has no way to know such a binding
  // needs to re-run on every animation tick - reading animating alone
  // would only catch the start/end of a gesture, not the frames between.
  Q_PROPERTY(int revision READ revision NOTIFY resultsChanged)

public:
  explicit HandController(const std::filesystem::path &hand_config_path, QObject *parent = nullptr);

  QString handName() const;
  QStringList chainNames() const;
  QString currentGesture() const { return _currentGesture; }
  bool animating() const { return _animation != nullptr; }
  int revision() const { return _revision; }

  Q_INVOKABLE int jointCountForChain(const QString &chainName) const;
  Q_INVOKABLE double jointAngleDegrees(const QString &chainName, int jointIndex) const;
  Q_INVOKABLE double jointMinDegrees(const QString &chainName, int jointIndex) const;
  Q_INVOKABLE double jointMaxDegrees(const QString &chainName, int jointIndex) const;

  // Snaps one joint straight to a target angle - no animation. For manual
  // control (and later, external sensor input); gestures always go
  // through playGesture() instead, which is never instant.
  Q_INVOKABLE void setJointAngleDegrees(const QString &chainName, int jointIndex, double degrees);

  // Joint positions (meters) for one chain, in the hand's own local
  // frame, in base-to-tip order - the same QVariantList-of-QVector3D
  // convention ArmController::jointPositions() uses, so QML renders both
  // the same way.
  Q_INVOKABLE QVariantList jointPositionsForChain(const QString &chainName) const;

  // Per-link midpoint (meters), length (meters), and orientation for one
  // chain - the same midpoint/length/rotation convention
  // ArmController::linkMidpoints()/linkLengths()/linkRotations() use for
  // the arm's own links, so QML can render a hand segment exactly the
  // way it already renders an arm link: one oriented, tapered cylinder
  // per entry. One entry per joint-to-joint gap, so N joints gives N-1
  // links, same as the arm.
  Q_INVOKABLE QVariantList linkMidpointsForChain(const QString &chainName) const;
  Q_INVOKABLE QVariantList linkLengthsForChain(const QString &chainName) const;
  Q_INVOKABLE QVariantList linkRotationsForChain(const QString &chainName) const;

  // The ONE segment linkMidpointsForChain()/etc. above can never
  // represent: the gap from this chain's own parent's current tip to
  // this chain's own first joint (e.g. a finger's metacarpal offset
  // from the wrist). Each returns a list of exactly one entry, or an
  // empty list if this chain has no parent (it's the tree's root) or
  // has no joints of its own.
  Q_INVOKABLE QVariantList rootSegmentMidpointForChain(const QString &chainName) const;
  Q_INVOKABLE QVariantList rootSegmentLengthForChain(const QString &chainName) const;
  Q_INVOKABLE QVariantList rootSegmentRotationForChain(const QString &chainName) const;

  // Starts a smooth animation toward a named gesture ("restPose",
  // "openPalm", "closedFist", "wave"), replacing any animation already
  // in progress. Unknown names are logged and ignored - a typo in a QML
  // button binding should do nothing, not crash.
  Q_INVOKABLE void playGesture(const QString &name);

signals:
  void resultsChanged();

private slots:
  void advanceAnimation();

private:
  struct RootSegment {
    bool present = false;
    Eigen::Vector3d midpoint = Eigen::Vector3d::Zero();
    double length = 0.0;
    QQuaternion rotation;
  };

  void recompute();
  std::vector<Eigen::Vector3d> chainPositions(const std::string &name) const;
  RootSegment rootSegmentFor(const std::string &name) const;
  std::vector<double> flatten(const std::map<std::string, std::vector<double>> &pose_by_chain) const;
  std::map<std::string, std::vector<double>> unflatten(const std::vector<double> &flat) const;
  // Never throws - every Q_INVOKABLE method must guard external (QML)
  // input itself and return a safe default on a bad chain name/index
  // rather than let an exception cross the QML call boundary.
  const HandChainConfig *findChainConfig(const std::string &name) const;
  void startAnimationTo(const std::vector<std::map<std::string, std::vector<double>>> &target_waypoints);

  static constexpr double kGestureSegmentSeconds = 0.9;
  static constexpr int kAnimationTickMilliseconds = 16; // ~60Hz

  HandConfig _config;
  KinematicTree _tree;
  HandGestures _gestures;
  std::vector<std::string> _chainOrder; // stable order for flatten()/unflatten()

  std::map<std::string, std::vector<double>> _jointAnglesRadians;
  std::map<std::string, std::vector<Eigen::Matrix4d>> _jointPoses; // cached forward kinematics, hand-local frame

  std::unique_ptr<RecordedTrajectory> _animation;
  QElapsedTimer _animationClock;
  QTimer _animationTicker;
  QString _currentGesture = QStringLiteral("restPose");
  int _revision = 0;
};

} // namespace robotarm
