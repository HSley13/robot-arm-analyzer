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
  // armName/jointCount/canvasCenter were CONSTANT before loadConfig()
  // (Phase 4) made them change at runtime - loading a different arm can
  // change its name, joint count, and reach (which canvasCenter pivots
  // around).
  Q_PROPERTY(QString armName READ armName NOTIFY resultsChanged)
  Q_PROPERTY(int jointCount READ jointCount NOTIFY resultsChanged)
  Q_PROPERTY(bool aiModeAvailable READ aiModeAvailable NOTIFY resultsChanged)
  Q_PROPERTY(QVector3D canvasCenter READ canvasCenter NOTIFY resultsChanged)

  // Phase 6: the arm's own joint kinematics/dynamics are entirely
  // computed in its own local frame, exactly as before - a mobile
  // base changing where that frame SITS in the world doesn't change
  // any torque or angle math at all, the same way a real arm's
  // dynamics don't care where its wheels parked it. baseX/baseY are
  // that world-space offset, applied only at the handful of
  // world-facing boundary points that need it (jointPositions(),
  // linkMidpoints(), payloadX()/Y()'s attached case, canvasCenter(),
  // and moveToTarget()'s input conversion) - see driveTo()'s own
  // comment for why it doesn't need to touch recompute() at all.
  Q_PROPERTY(double baseX READ baseX NOTIFY resultsChanged)
  Q_PROPERTY(double baseY READ baseY NOTIFY resultsChanged)
  // Feedback for loadConfig()/saveConfig() - success or failure, since
  // both can fail (bad file, bad path) and the UI needs to tell the user
  // why rather than silently doing nothing.
  Q_PROPERTY(QString configMessage READ configMessage NOTIFY resultsChanged)

  Q_PROPERTY(double payloadKg READ payloadKg WRITE setPayloadKg NOTIFY resultsChanged)
  // Phase 6: the payload is a real object sitting in the arm's plane,
  // not something permanently glued to the gripper - placePayload()
  // sets where it rests; closing the gripper only picks it up if the
  // end effector is actually there (see setGripperOpen()). payloadX/Y
  // report wherever it currently IS: its own resting spot while free,
  // or the end effector's live position while attached - so QML can
  // bind one pair of numbers for "where's the payload" regardless of
  // which state it's in.
  Q_PROPERTY(double payloadX READ payloadX NOTIFY resultsChanged)
  Q_PROPERTY(double payloadY READ payloadY NOTIFY resultsChanged)
  Q_PROPERTY(bool payloadAttached READ payloadAttached NOTIFY resultsChanged)
  // Closed by default (no pre-existing demo behavior depended on
  // starting open). Closing it only attaches the payload if the end
  // effector is within pickup range of where the payload currently
  // rests - closing it elsewhere just closes an empty gripper, the
  // way a real one would. Opening it while holding the payload drops
  // it exactly where it is.
  Q_PROPERTY(bool gripperOpen READ gripperOpen WRITE setGripperOpen NOTIFY resultsChanged)
  Q_PROPERTY(bool aiMode READ aiMode WRITE setAiMode NOTIFY resultsChanged)
  // Purely a UI toggle (no physics behind it, unlike every other bool
  // property above) - stored here anyway because ControlsPanel.qml's
  // switch and ArmView3D.qml's workspace-envelope disc are siblings
  // under Main.qml, not parent/child, and armController is the one
  // thing both already have access to.
  Q_PROPERTY(bool showWorkspaceEnvelope READ showWorkspaceEnvelope WRITE setShowWorkspaceEnvelope NOTIFY resultsChanged)

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
  // Was CONSTANT before setJointTorqueLimit() (Phase 4) made limits
  // user-editable from the UI, ahead of saving them to a config file.
  Q_PROPERTY(QVariantList jointTorqueLimits READ jointTorqueLimits NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointColors READ jointColors NOTIFY resultsChanged)
  // Each joint's own mass, kg - editable via setJointMass() so the
  // arm itself can be scaled up or down (a tiny precision arm vs. a
  // massive industrial one), not just the payload it's carrying.
  Q_PROPERTY(QVariantList jointMassesKg READ jointMassesKg NOTIFY resultsChanged)

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

  // Phase 6: "how much more could this arm safely lift right now,
  // before any joint exceeds its torque limit" - a real, safety-
  // relevant number (the kind a spec sheet's load-moment diagram
  // answers), not just a comparison of two engines. Always computed
  // via the exact engine regardless of aiMode, since this is the one
  // number in the UI that's meant to be trusted outright.
  Q_PROPERTY(double currentMaxSafePayloadKg READ currentMaxSafePayloadKg NOTIFY resultsChanged)

  // runPayloadScan() results: a swept "payload vs. reach" curve (the
  // same concept as a manufacturer's derating curve), computed twice -
  // once through the exact engine, once through the AI surrogate - so
  // scanExactMilliseconds/scanAiMilliseconds give a genuine, measured
  // answer to "why use the AI surrogate at all", instead of the
  // Exact/AI toggle's single-pose comparison.
  Q_PROPERTY(QVariantList scanReachMeters READ scanReachMeters NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList scanExactPayloadKg READ scanExactPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList scanAiPayloadKg READ scanAiPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(double scanExactMilliseconds READ scanExactMilliseconds NOTIFY resultsChanged)
  Q_PROPERTY(double scanAiMilliseconds READ scanAiMilliseconds NOTIFY resultsChanged)
  Q_PROPERTY(bool scanAiAvailable READ scanAiAvailable NOTIFY resultsChanged)
  Q_PROPERTY(int scanSampleCount READ scanSampleCount NOTIFY resultsChanged)

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
  QVector3D canvasCenter() const;
  double baseX() const { return _baseX; }
  double baseY() const { return _baseY; }

  double payloadKg() const { return _payloadKg; }
  void setPayloadKg(double kg);
  double payloadX() const;
  double payloadY() const;
  bool payloadAttached() const { return _payloadAttached; }
  bool gripperOpen() const { return _gripperOpen; }
  void setGripperOpen(bool open);
  bool showWorkspaceEnvelope() const { return _showWorkspaceEnvelope; }
  void setShowWorkspaceEnvelope(bool show);
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
  QVariantList jointMassesKg() const;

  QVariantList gravityTorques() const;
  QVariantList inertiaTorques() const;
  QVariantList coriolisTorques() const;

  QVariantList jointPositions() const;
  QVariantList linkMidpoints() const;
  QVariantList linkLengths() const;
  QVariantList linkRotations() const;

  double currentMaxSafePayloadKg() const { return _currentMaxSafePayloadKg; }
  QVariantList scanReachMeters() const;
  QVariantList scanExactPayloadKg() const;
  QVariantList scanAiPayloadKg() const;
  double scanExactMilliseconds() const { return _scanExactMilliseconds; }
  double scanAiMilliseconds() const { return _scanAiMilliseconds; }
  bool scanAiAvailable() const { return _scanAiAvailable; }
  int scanSampleCount() const { return static_cast<int>(_scanReachMeters.size()); }

  Q_INVOKABLE void setJointAngleDegrees(int index, double degrees);

  // Click-to-reach inverse kinematics: solves for joint angles that
  // bring the end effector to (xMeters, yMeters) in the base frame's
  // XY plane (z is always 0 - every arm this project models is
  // planar, since every joint rotates about the same Z axis, so a
  // third coordinate wouldn't mean anything here), starting the
  // search from the CURRENT pose as its initial guess. An unreachable
  // target still updates the pose to whatever solve_inverse_kinematics()
  // converges on - the closest reachable approach, not a no-op -
  // exactly like it would for a real arm straining toward something
  // just out of reach.
  Q_INVOKABLE void moveToTarget(double xMeters, double yMeters);

  // Drives the base to a new world position - instant on the backend
  // (just two numbers), with QML animating the visual motion via a
  // Behavior the same way the gripper fingers already animate their
  // open/close. Doesn't touch recompute() or any joint angle: the
  // arm's own kinematics are defined entirely relative to its base,
  // so moving the base doesn't change what pose the arm is holding,
  // only where that pose sits in the world - exactly like a real
  // mobile manipulator's arm controller not needing to know about the
  // chassis's navigation stack.
  Q_INVOKABLE void driveTo(double worldX, double worldY);

  // Convenience for the common two-stage demo motion: parks the base
  // so the payload ends up half the arm's own reach directly ahead of
  // it (not exactly on top of the base's own origin, which would
  // trivially satisfy "in reach" without the arm needing to extend at
  // all) - close enough that moveToTarget(payloadX, payloadY) or the
  // "Go to payload" button can then reach the rest of the way.
  Q_INVOKABLE void driveNearPayload();

  // Sets where the payload rests when nothing is holding it. A no-op
  // while payloadAttached() is true - you can't teleport something
  // you're currently gripping to a different spot; open the gripper
  // to put it down first. z is always 0 for the same planar reason
  // moveToTarget()'s target is.
  Q_INVOKABLE void placePayload(double xMeters, double yMeters);

  // Only take effect when trajectoryScrubMode is off - while scrubbing,
  // velocity/acceleration are read off the trajectory instead.
  Q_INVOKABLE void setJointVelocityDegrees(int index, double degreesPerSecond);
  Q_INVOKABLE void setJointAccelerationDegrees(int index, double degreesPerSecondSquared);
  // Appends the current pose (jointAnglesDegrees' underlying target) as
  // a new waypoint, extending the recording by one more segment. Only
  // takes effect while trajectoryScrubMode is on.
  Q_INVOKABLE void recordWaypoint();

  // Edits this joint's torque_limit in place (clamped to the same
  // "must be positive" rule parse_arm_config() enforces on load) so it
  // can be tuned from the UI before saveConfig() writes it out.
  Q_INVOKABLE void setJointTorqueLimit(int index, double limit);

  // Edits this joint's DH `a` (link length) in place and rebuilds _arm
  // from the updated config - unlike setJointAngleDegrees(), this
  // changes the arm's actual geometry, not just its current pose.
  // Re-derives this link's inertial data as LinkInertial::rod(mass,
  // meters) regardless of whether the loaded config originally gave it
  // an explicit inertia tensor - a documented simplification, not an
  // oversight: every config this project ships uses the default rod,
  // and there's no way to tell post-parse whether a given JointConfig's
  // `inertial` came from that default or an explicit override. Also
  // turns aiMode off: a trained surrogate implicitly assumes the
  // geometry it was trained on, and has no way to be told a link got
  // longer or shorter, so its predictions would silently stop matching
  // reality the moment this changes.
  Q_INVOKABLE void setJointLength(int index, double meters);

  // Edits this joint's own mass in place - the arm's weight, not the
  // payload it's carrying, so the whole arm can be scaled to simulate
  // something much lighter or heavier than the shipped defaults.
  // Rejects negative values only (matching parse_arm_config()'s own
  // rule - a massless link, mass 0, is valid). Re-derives this link's
  // inertial data as LinkInertial::rod(kg, a) the same way
  // setJointLength() does, and for the same reason turns aiMode off:
  // the surrogate's training data baked in the ORIGINAL masses, so
  // its predictions would silently stop matching reality the moment
  // a mass changes. Geometry (_arm) is untouched - only _links/
  // _inertialLinks need rebuilding.
  Q_INVOKABLE void setJointMass(int index, double kg);

  // Appends a new revolute joint to the end of the chain - a real
  // topology change, not just an edited existing joint. Defaults
  // continue the tapering convention the 3D view already draws (each
  // new joint shorter/lighter/weaker than the one before it): 0.7x
  // the previous last joint's length, 0.6x its mass, 0.5x its torque
  // limit (or fixed defaults - 0.5m/1.0kg/20 N*m - if this is the very
  // first joint on an otherwise-empty arm, which shouldn't normally
  // happen but costs nothing to handle). None of these numbers are
  // meant to be physically authoritative - every new joint's length
  // and torque limit stay editable afterward via the existing
  // controls, same as any other joint.
  //
  // Invalidates the same things loadConfig() does for a joint-count
  // change: aiMode (surrogate joint count no longer matches), any
  // payload scan (described the old shape), and trajectory scrubbing
  // (a recording's waypoints are sized to the OLD joint count and
  // would no longer match _jointAnglesRadians).
  Q_INVOKABLE void addJoint();

  // Removes the LAST joint in the chain - mirrors addJoint(), not
  // arbitrary-index removal, so nothing downstream needs renumbering.
  // Refuses to go below 1 joint (an arm needs at least one). Same
  // invalidation as addJoint().
  Q_INVOKABLE void removeLastJoint();

  QString configMessage() const { return _configMessage; }

  // Replaces the current arm entirely with the one loaded from path:
  // geometry, mass, inertial data, torque limits, joint count. Resets
  // pose/payload/mode state to defaults rather than trying to carry it
  // over, since none of it necessarily still makes sense for a
  // different arm (a different joint count, in particular). Returns
  // false (and sets configMessage) instead of throwing, so a bad file
  // reports an error in the UI rather than crashing it.
  Q_INVOKABLE bool loadConfig(const QString &path);

  // Writes the current arm (including any edits from setJointTorqueLimit)
  // to path. Returns false (and sets configMessage) on failure instead of
  // throwing, matching loadConfig()'s convention.
  Q_INVOKABLE bool saveConfig(const QString &path);

  // Sweeps the arm from fully extended to progressively folded (the
  // shoulder stays at its current angle; every other joint bends
  // together, so the end effector's distance from the base - its
  // "reach" - shrinks continuously), computing max_safe_payload() at
  // each sampled reach through the exact engine and, if the current
  // mode has a matching surrogate, through it as well, timing each
  // sweep as a whole. Clamped to [2, 300] samples - too few isn't a
  // curve, and too many just makes the UI wait without showing
  // anything new past a certain density.
  Q_INVOKABLE void runPayloadScan(int sampleCount);

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
  // payloadKg() with the pickup state applied: 0 unless the payload is
  // actually attached - closing the gripper somewhere else in the
  // plane doesn't magically apply its weight. Every torque calculation
  // in recompute() reads this instead of _payloadKg directly - one
  // place decides this so a future calculation can't forget to check
  // and silently apply a payload that isn't actually held.
  double effectivePayloadKg() const { return _payloadAttached ? _payloadKg : 0.0; }

  // Recomputes the default resting spot for a freshly loaded/constructed
  // arm: comfortably within reach (45% of total length) and a little
  // below shoulder height, so it reads as "sitting on the ground" next
  // to the pedestal rather than floating at the arm's own origin.
  void resetPayloadPosition();

  // Duration of each segment in _recordedTrajectory - between any two
  // consecutive waypoints, however many have been recorded. The whole
  // recording's duration scales with waypoint count; this is per-segment.
  static constexpr double kTrajectoryDurationSeconds = 2.0;

  // Upper bound max_safe_payload() searches up to, for
  // currentMaxSafePayloadKg and runPayloadScan() - far past anything
  // the payload slider itself allows (0-5kg), so "how much could this
  // arm really hold" isn't artificially capped at the slider's own
  // demo range.
  static constexpr double kPayloadSearchCeilingKg = 200.0;

  // How close the end effector must be to the payload's resting spot
  // for closing the gripper to actually pick it up. Generous enough
  // that solve_inverse_kinematics()'s own convergence tolerance (far
  // tighter than this) never causes a "just missed it" surprise, tight
  // enough that closing the gripper somewhere else in the plane
  // obviously doesn't attach it.
  static constexpr double kPickupRadiusMeters = 0.15;

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
  double _payloadX = 0.0; // resting position while NOT attached - see resetPayloadPosition()
  double _payloadY = 0.0;
  bool _payloadAttached = false;
  bool _gripperOpen = true; // open by default - a ready-to-approach hand, see the Q_PROPERTY doc comment above
  double _baseX = 0.0; // world-space offset of the arm's own local frame - see driveTo()
  double _baseY = 0.0;
  bool _showWorkspaceEnvelope = false;
  bool _aiMode = false;
  bool _dynamicMode = false;

  bool _trajectoryScrubMode = false;
  double _trajectoryProgress = 0.0;
  // Rebuilt from [rest, current pose] every time scrubbing is turned
  // on (discarding any previous recording) - null when scrubbing has
  // never been enabled, since RecordedTrajectory has no empty state.
  std::unique_ptr<RecordedTrajectory> _recordedTrajectory;

  TorqueBreakdown _breakdown;

  std::vector<Eigen::Vector3d> _jointPositions; // base origin + one per joint, LOCAL frame - see canvasCenter()
  std::vector<double> _jointTorques;
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
