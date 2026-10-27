#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QVariantList>
#include <QVector3D>

#include "robotarm/arm_config.hpp"
#include "robotarm/arm_model.hpp"
#include "robotarm/dynamics.hpp"
#include "robotarm/recorded_trajectory.hpp"
#include "robotarm/statics.hpp"
#include "robotarm/surrogate.hpp"
#include "robotarm/trajectory.hpp"

namespace robotarm {

// QML-facing backend for the robot arm view: owns the ArmModel/LinkMass
// data for one arm config, holds the current joint angles/payload/mode,
// and recomputes torques, joint positions, and threshold colors whenever
// any of those change. QML only ever sees plain Qt types (QVariantList,
// QVector3D, QString, double, bool, int) - never the core/ml C++ types
// directly, matching the plan's "three pieces only share data" goal.
class ArmController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString armName READ armName CONSTANT)
  Q_PROPERTY(int jointCount READ jointCount CONSTANT)
  Q_PROPERTY(bool aiModeAvailable READ aiModeAvailable NOTIFY resultsChanged)
  Q_PROPERTY(QVector3D canvasCenter READ canvasCenter CONSTANT)

  Q_PROPERTY(double payloadKg READ payloadKg WRITE setPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(bool aiMode READ aiMode WRITE setAiMode NOTIFY resultsChanged)

  // Dynamic mode (Phase 2): when on, jointVelocitiesDegrees/
  // jointAccelerationsDegrees feed inverse_dynamics() (or the dynamic AI
  // surrogate) instead of the static (at-rest) calculation.
  Q_PROPERTY(bool dynamicMode READ dynamicMode WRITE setDynamicMode NOTIFY resultsChanged)
  // Trajectory scrubbing (Phase 2) and multi-waypoint recording/playback
  // (Phase 4): per the roadmap, "scrub through a simple trajectory"
  // instead of specifying velocity/acceleration directly, and "record/
  // scrub through a sequence of poses". Enabling trajectoryScrubMode
  // seeds a RecordedTrajectory with [rest, current pose]; recordWaypoint()
  // appends the current pose as another stop. Dragging trajectoryProgress
  // (0..1, spanning the WHOLE recording, however many waypoints) moves
  // the pose along it, with velocity/acceleration read off automatically.
  // Implies dynamicMode.
  Q_PROPERTY(bool trajectoryScrubMode READ trajectoryScrubMode WRITE setTrajectoryScrubMode NOTIFY resultsChanged)
  Q_PROPERTY(double trajectoryProgress READ trajectoryProgress WRITE setTrajectoryProgress NOTIFY resultsChanged)
  Q_PROPERTY(int trajectoryWaypointCount READ trajectoryWaypointCount NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList jointAnglesDegrees READ jointAnglesDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointVelocitiesDegrees READ jointVelocitiesDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointAccelerationsDegrees READ jointAccelerationsDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorques READ jointTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorqueLimits READ jointTorqueLimits CONSTANT)
  Q_PROPERTY(QVariantList jointColors READ jointColors NOTIFY resultsChanged)

  // The torque breakdown (Phase 2): gravity/inertia/Coriolis, per the
  // roadmap's NovaSolver lesson. Always valid even outside dynamic mode
  // (inertia/coriolis are simply ~0 at rest, leaving gravity == total).
  Q_PROPERTY(QVariantList gravityTorques READ gravityTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList inertiaTorques READ inertiaTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList coriolisTorques READ coriolisTorques NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList jointPositions READ jointPositions NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkMidpoints READ linkMidpoints NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkLengths READ linkLengths NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkRotations READ linkRotations NOTIFY resultsChanged)

public:
  // surrogate_path/dynamic_surrogate_path may point to a file that
  // doesn't exist - AI mode is then simply unavailable for that mode
  // (aiModeAvailable() returns false while in it) rather than throwing,
  // since not every arm has a trained surrogate yet.
  explicit ArmController(const std::filesystem::path &config_path, const std::filesystem::path &surrogate_path,
                          const std::filesystem::path &dynamic_surrogate_path = {}, QObject *parent = nullptr);

  QString armName() const;
  int jointCount() const { return static_cast<int>(_arm.num_joints()); }
  bool aiModeAvailable() const;
  QVector3D canvasCenter() const { return _canvasCenter; }

  double payloadKg() const { return _payloadKg; }
  void setPayloadKg(double kg);
  bool aiMode() const { return _aiMode; }
  void setAiMode(bool enabled);

  bool dynamicMode() const { return _dynamicMode; }
  void setDynamicMode(bool enabled);
  bool trajectoryScrubMode() const { return _trajectoryScrubMode; }
  void setTrajectoryScrubMode(bool enabled);
  double trajectoryProgress() const { return _trajectoryProgress; }
  void setTrajectoryProgress(double progress);
  int trajectoryWaypointCount() const;

  QVariantList jointAnglesDegrees() const;
  QVariantList jointVelocitiesDegrees() const;
  QVariantList jointAccelerationsDegrees() const;
  QVariantList jointTorques() const;
  QVariantList jointTorqueLimits() const;
  QVariantList jointColors() const;

  QVariantList gravityTorques() const;
  QVariantList inertiaTorques() const;
  QVariantList coriolisTorques() const;

  QVariantList jointPositions() const;
  QVariantList linkMidpoints() const;
  QVariantList linkLengths() const;
  QVariantList linkRotations() const;

  Q_INVOKABLE void setJointAngleDegrees(int index, double degrees);
  // Only take effect when trajectoryScrubMode is off - while scrubbing,
  // velocity/acceleration are read off the trajectory instead.
  Q_INVOKABLE void setJointVelocityDegrees(int index, double degreesPerSecond);
  Q_INVOKABLE void setJointAccelerationDegrees(int index, double degreesPerSecondSquared);
  // Appends the current pose (jointAnglesDegrees' underlying target) as
  // a new waypoint, extending the recording by one more segment. Only
  // takes effect while trajectoryScrubMode is on.
  Q_INVOKABLE void recordWaypoint();

signals:
  void resultsChanged();

private:
  void recompute();
  // The current position/velocity/acceleration, trajectory-aware: while
  // scrubbing, all three are read off _recordedTrajectory; otherwise
  // position is the directly-set pose, and velocity/acceleration are
  // zero outside dynamicMode or the manually-set values inside it. One
  // place decides this so recompute() and the *Degrees() accessors can't
  // disagree with each other about what "current" means.
  TrajectoryState effectiveState() const;

  // Duration of each segment in _recordedTrajectory - between any two
  // consecutive waypoints, however many have been recorded. The whole
  // recording's duration scales with waypoint count; this is per-segment.
  static constexpr double kTrajectoryDurationSeconds = 2.0;

  ArmConfig _config;
  ArmModel _arm;
  std::vector<LinkMass> _links;
  std::vector<LinkInertial> _inertialLinks;
  std::unique_ptr<Surrogate> _surrogate;
  std::unique_ptr<Surrogate> _dynamicSurrogate;

  std::vector<double> _jointAnglesRadians;
  std::vector<double> _jointVelocitiesRadians;
  std::vector<double> _jointAccelerationsRadians;
  double _payloadKg = 0.0;
  bool _aiMode = false;
  bool _dynamicMode = false;

  bool _trajectoryScrubMode = false;
  double _trajectoryProgress = 0.0;
  // Rebuilt from [rest, current pose] every time scrubbing is turned
  // on (discarding any previous recording) - null when scrubbing has
  // never been enabled, since RecordedTrajectory has no empty state.
  std::unique_ptr<RecordedTrajectory> _recordedTrajectory;

  TorqueBreakdown _breakdown;

  std::vector<Eigen::Vector3d> _jointPositions; // base origin + one per joint
  std::vector<double> _jointTorques;
  QVector3D _canvasCenter;
};

} // namespace robotarm
