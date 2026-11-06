#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QStringList>
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

// QML-facing backend: owns one ArmModel/LinkMass config, current pose/
// payload/mode, and recomputes torques/positions/colors on change. Only
// exposes plain Qt types to QML, never the core/ml C++ types directly.
class ArmController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString armName READ armName NOTIFY resultsChanged)
  Q_PROPERTY(int jointCount READ jointCount NOTIFY resultsChanged)
  Q_PROPERTY(bool aiModeAvailable READ aiModeAvailable NOTIFY resultsChanged)
  Q_PROPERTY(QVector3D canvasCenter READ canvasCenter NOTIFY resultsChanged)
  Q_PROPERTY(QString configMessage READ configMessage NOTIFY resultsChanged)
  // How far the arm's base is leaning forward (degrees) - the humanoid's
  // hip bend. Every torque, capacity and scan number accounts for it by
  // turning gravity in the arm's frame; joint positions stay in the
  // arm's own (torso-attached) frame.
  Q_PROPERTY(double baseTiltDegrees READ baseTiltDegrees WRITE setBaseTiltDegrees NOTIFY resultsChanged)

  Q_PROPERTY(double payloadKg READ payloadKg WRITE setPayloadKg NOTIFY resultsChanged)
  // effectivePayloadKg() is 0 while the gripper is open, regardless of
  // payloadKg - open it and drag the slider, nothing moves.
  Q_PROPERTY(bool gripperOpen READ gripperOpen WRITE setGripperOpen NOTIFY resultsChanged)
  Q_PROPERTY(bool aiMode READ aiMode WRITE setAiMode NOTIFY resultsChanged)
  Q_PROPERTY(bool dynamicMode READ dynamicMode WRITE setDynamicMode NOTIFY resultsChanged)
  Q_PROPERTY(bool trajectoryScrubMode READ trajectoryScrubMode WRITE setTrajectoryScrubMode NOTIFY resultsChanged)
  Q_PROPERTY(double trajectoryProgress READ trajectoryProgress WRITE setTrajectoryProgress NOTIFY resultsChanged)
  Q_PROPERTY(int trajectoryWaypointCount READ trajectoryWaypointCount NOTIFY resultsChanged)

  Q_PROPERTY(QStringList jointNames READ jointNames NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointMinDegrees READ jointMinDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointMaxDegrees READ jointMaxDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointAnglesDegrees READ jointAnglesDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointVelocitiesDegrees READ jointVelocitiesDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointAccelerationsDegrees READ jointAccelerationsDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorques READ jointTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorqueLimits READ jointTorqueLimits NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointColors READ jointColors NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointMassesKg READ jointMassesKg NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList gravityTorques READ gravityTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList inertiaTorques READ inertiaTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList coriolisTorques READ coriolisTorques NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList jointPositions READ jointPositions NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkMidpoints READ linkMidpoints NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkLengths READ linkLengths NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkRotations READ linkRotations NOTIFY resultsChanged)
  // Full orientation of every frame in jointPositions (same length: the
  // base frame first, then one per joint) - unlike linkRotations, which
  // only aligns +Y with a link and throws away the roll about it. The 3D
  // view needs the roll to mount the hand and orient each limb shell.
  Q_PROPERTY(QVariantList frameRotations READ frameRotations NOTIFY resultsChanged)

  // Always via the exact engine, regardless of aiMode - the one number
  // meant to be trusted outright.
  Q_PROPERTY(double currentMaxSafePayloadKg READ currentMaxSafePayloadKg NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList scanReachMeters READ scanReachMeters NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList scanExactPayloadKg READ scanExactPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList scanAiPayloadKg READ scanAiPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(double scanExactMilliseconds READ scanExactMilliseconds NOTIFY resultsChanged)
  Q_PROPERTY(double scanAiMilliseconds READ scanAiMilliseconds NOTIFY resultsChanged)
  Q_PROPERTY(bool scanAiAvailable READ scanAiAvailable NOTIFY resultsChanged)
  Q_PROPERTY(int scanSampleCount READ scanSampleCount NOTIFY resultsChanged)

public:
  explicit ArmController(const std::filesystem::path &config_path, const std::filesystem::path &surrogate_path,
                          const std::filesystem::path &dynamic_surrogate_path = {}, QObject *parent = nullptr);

  QString armName() const;
  int jointCount() const { return static_cast<int>(_arm.num_joints()); }
  bool aiModeAvailable() const;
  QVector3D canvasCenter() const { return _canvasCenter; }
  double baseTiltDegrees() const { return _baseTiltRadians * 180.0 / 3.14159265358979323846; }
  void setBaseTiltDegrees(double degrees);

  // C++-side access for planners (TaskController): the live config (with
  // any lengths/masses edited in the UI), and a whole-pose update with a
  // single recompute instead of one per joint.
  const ArmConfig &config() const { return _config; }
  std::vector<double> jointAnglesRadians() const { return _jointAnglesRadians; }
  void setJointAnglesRadians(const std::vector<double> &radians);

  double payloadKg() const { return _payloadKg; }
  void setPayloadKg(double kg);
  bool gripperOpen() const { return _gripperOpen; }
  void setGripperOpen(bool open);
  bool aiMode() const { return _aiMode; }
  void setAiMode(bool enabled);

  bool dynamicMode() const { return _dynamicMode; }
  void setDynamicMode(bool enabled);
  bool trajectoryScrubMode() const { return _trajectoryScrubMode; }
  void setTrajectoryScrubMode(bool enabled);
  double trajectoryProgress() const { return _trajectoryProgress; }
  void setTrajectoryProgress(double progress);
  int trajectoryWaypointCount() const;

  QStringList jointNames() const;
  QVariantList jointMinDegrees() const;
  QVariantList jointMaxDegrees() const;
  QVariantList jointAnglesDegrees() const;
  QVariantList jointVelocitiesDegrees() const;
  QVariantList jointAccelerationsDegrees() const;
  QVariantList jointTorques() const;
  QVariantList jointTorqueLimits() const;
  QVariantList jointColors() const;
  QVariantList jointMassesKg() const;

  QVariantList gravityTorques() const;
  QVariantList inertiaTorques() const;
  QVariantList coriolisTorques() const;

  QVariantList jointPositions() const;
  QVariantList linkMidpoints() const;
  QVariantList linkLengths() const;
  QVariantList linkRotations() const;
  QVariantList frameRotations() const;

  double currentMaxSafePayloadKg() const { return _currentMaxSafePayloadKg; }
  QVariantList scanReachMeters() const;
  QVariantList scanExactPayloadKg() const;
  QVariantList scanAiPayloadKg() const;
  double scanExactMilliseconds() const { return _scanExactMilliseconds; }
  double scanAiMilliseconds() const { return _scanAiMilliseconds; }
  bool scanAiAvailable() const { return _scanAiAvailable; }
  int scanSampleCount() const { return static_cast<int>(_scanReachMeters.size()); }

  // Clamped into the joint's configured min_deg..max_deg travel.
  Q_INVOKABLE void setJointAngleDegrees(int index, double degrees);

  // Solves for joint angles reaching (xMeters, yMeters) in the base
  // frame's XY plane, from the current pose. An unreachable target
  // still updates to the closest approach, not a no-op.
  Q_INVOKABLE void moveToTarget(double xMeters, double yMeters);

  Q_INVOKABLE void setJointVelocityDegrees(int index, double degreesPerSecond);
  Q_INVOKABLE void setJointAccelerationDegrees(int index, double degreesPerSecondSquared);
  Q_INVOKABLE void recordWaypoint();

  Q_INVOKABLE void setJointTorqueLimit(int index, double limit);

  // Edits this joint's DH `a` and rebuilds _arm - a geometry change,
  // not just a pose change. Turns aiMode off: the surrogate assumes
  // fixed geometry.
  Q_INVOKABLE void setJointLength(int index, double meters);

  // Turns aiMode off for the same reason setJointLength() does - the
  // surrogate's training data baked in the original masses.
  Q_INVOKABLE void setJointMass(int index, double kg);

  Q_INVOKABLE void addJoint();
  Q_INVOKABLE void removeLastJoint();

  QString configMessage() const { return _configMessage; }

  Q_INVOKABLE bool loadConfig(const QString &path);
  Q_INVOKABLE bool saveConfig(const QString &path);

  // Sweeps from fully extended to folded, computing max_safe_payload()
  // at each sampled reach through the exact engine and (if available)
  // the AI surrogate, timing each sweep as a whole.
  Q_INVOKABLE void runPayloadScan(int sampleCount);

signals:
  void resultsChanged();

private:
  void recompute();
  // Clamps a joint value (radians) into that joint's configured travel.
  double clampToLimits(size_t index, double radians) const;
  Eigen::Vector3d gravity() const;
  std::vector<double> surrogateAngles(std::vector<double> angles) const;
  TrajectoryState effectiveState() const;
  double effectivePayloadKg() const { return _gripperOpen ? 0.0 : _payloadKg; }

  static constexpr double kTrajectoryDurationSeconds = 2.0;
  static constexpr double kPayloadSearchCeilingKg = 200.0;

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
  bool _gripperOpen = false;
  bool _aiMode = false;
  bool _dynamicMode = false;
  double _baseTiltRadians = 0.0;

  bool _trajectoryScrubMode = false;
  double _trajectoryProgress = 0.0;
  std::unique_ptr<RecordedTrajectory> _recordedTrajectory;

  TorqueBreakdown _breakdown;

  std::vector<Eigen::Vector3d> _jointPositions;
  std::vector<Eigen::Matrix3d> _frameRotations;
  std::vector<double> _jointTorques;
  QVector3D _canvasCenter;
  QString _configMessage;

  double _currentMaxSafePayloadKg = 0.0;
  std::vector<double> _scanReachMeters;
  std::vector<double> _scanExactPayloadKg;
  std::vector<double> _scanAiPayloadKg;
  double _scanExactMilliseconds = 0.0;
  double _scanAiMilliseconds = 0.0;
  bool _scanAiAvailable = false;
};

} // namespace robotarm
