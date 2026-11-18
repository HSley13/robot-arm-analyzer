#pragma once

#include <deque>
#include <functional>
#include <vector>

#include <QObject>
#include <QtQml/qqmlregistration.h>
#include <QString>
#include <QStringList>
#include <QElapsedTimer>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "robotarm/intent.hpp"
#include "robotarm/pick_planner.hpp"

class QQmlEngine;
class QJSEngine;

namespace robotarm {

// The humanoid's "do something useful" layer: the objects lying around
// the scene, the body's own degrees of freedom (waist turn, lift height,
// hip bend), and animated pick-up / put-down sequences that drive
// ArmController (arm joints + payload) and HandController (fingers).
//
// It owns no physics of its own - every pose comes from plan_grasp()
// (core/pick_planner), and while an object is held its mass is pushed
// into ArmController as the payload, so the torque gauges, safety colors
// and capacity number all describe what the robot is actually carrying.
//
// Animation is a queue of Steps (a target body pose + a duration + an
// optional action on arrival), advanced by a QTimer in the app or by
// calling advance() directly in tests - no wall clock needed to test it.
//
// World conventions are pick_planner.hpp's: +Y up, floor at y = 0, the
// waist axis through the origin, the robot facing +X at yaw 0.
class TaskController : public QObject {
  Q_OBJECT
  // QML: `import Humanoid.Backend`, then use TaskController.<property> - one
  // shared instance (the one main() made, see qml_singleton.hpp).
  QML_ELEMENT
  QML_SINGLETON
  Q_PROPERTY(QVariantList objects READ objects NOTIFY objectsChanged)
  Q_PROPERTY(QStringList objectTypes READ objectTypes CONSTANT)
  Q_PROPERTY(int selectedId READ selectedId WRITE selectObject NOTIFY objectsChanged)
  Q_PROPERTY(int heldId READ heldId NOTIFY objectsChanged)
  Q_PROPERTY(QVariantMap heldObject READ heldObject NOTIFY objectsChanged)

  Q_PROPERTY(bool busy READ busy NOTIFY stateChanged)
  // True while the body is still moving toward the last reachTo() goal (or
  // the left arm toward its own).
  Q_PROPERTY(bool reaching READ reaching NOTIFY stateChanged)
  Q_PROPERTY(QString status READ status NOTIFY stateChanged)
  // What the next click in the 3D view does: "" (select an object),
  // "place" (drop a new placeType/placeMassKg object there), "move" (move
  // the selected object there), "putDown" (put the held object there).
  Q_PROPERTY(QString clickMode READ clickMode WRITE setClickMode NOTIFY stateChanged)
  Q_PROPERTY(QString placeType READ placeType WRITE setPlaceType NOTIFY stateChanged)
  Q_PROPERTY(double placeMassKg READ placeMassKg WRITE setPlaceMassKg NOTIFY stateChanged)

  Q_PROPERTY(double waistYawDegrees READ waistYawDegrees WRITE setWaistYawDegrees NOTIFY bodyChanged)
  Q_PROPERTY(double hipHeight READ hipHeight WRITE setHipHeight NOTIFY bodyChanged)
  Q_PROPERTY(double hipBendDegrees READ hipBendDegrees WRITE setHipBendDegrees NOTIFY bodyChanged)
  // The hand turned about its own long axis (the forearm's twist): 0 =
  // thumb up, palm toward the body; + turns the palm up. See setHandRoll.
  Q_PROPERTY(double handRollDegrees READ handRollDegrees NOTIFY bodyChanged)
  // Body dimensions and ranges for the 3D model and the sliders.
  Q_PROPERTY(QVariantMap body READ body CONSTANT)
  // The tables, coffee tables and counters standing around:
  // [{id, kind, label, x, z, width, depth, height, x0, x1, z0, z1}].
  Q_PROPERTY(QVariantList tables READ tables NOTIFY tablesChanged)
  Q_PROPERTY(QStringList tableKinds READ tableKinds CONSTANT)
  // Distance between the industrial gripper's fingers (m): wide open, or
  // closed onto whatever it's holding.
  Q_PROPERTY(double gripperWidth READ gripperWidth NOTIFY stateChanged)
  // How closed the hand (or gripper) is being asked to be by live input,
  // 0 = open .. 1 = closed (see setGrip).
  Q_PROPERTY(double grip READ grip NOTIFY stateChanged)
  // Help for live control (guidedReach): guess the object the person is
  // reaching for, pull the hand toward it as the guess firms up, and
  // finish the grab when they close their hand near it. On by default.
  Q_PROPERTY(bool assist READ assist WRITE setAssist NOTIFY stateChanged)
  // The guess: the object's id (-1 = none), how sure (0..1), and how
  // strongly the hand is being pulled toward it right now (0..1).
  // While dragging the hand with the mouse: the object in reach (see
  // grabOrRelease), brighter the closer.
  Q_PROPERTY(int intentId READ intentId NOTIFY intentChanged)
  Q_PROPERTY(double intentConfidence READ intentConfidence NOTIFY intentChanged)
  Q_PROPERTY(double assistStrength READ assistStrength NOTIFY intentChanged)

  // The left arm, when there is one (setLeftArm): its own ArmController
  // (joints, loads - the same analysis as the right arm's) and
  // HandController (fingers), what it holds, its hand's twist and grip.
  Q_PROPERTY(robotarm::ArmController *leftArm READ leftArm NOTIFY leftArmChanged)
  Q_PROPERTY(robotarm::HandController *leftHand READ leftHand NOTIFY leftArmChanged)
  Q_PROPERTY(int leftHeldId READ leftHeldId NOTIFY objectsChanged)
  Q_PROPERTY(QVariantMap leftHeldObject READ leftHeldObject NOTIFY objectsChanged)
  Q_PROPERTY(double leftHandRollDegrees READ leftHandRollDegrees NOTIFY bodyChanged)
  Q_PROPERTY(double leftGrip READ leftGrip NOTIFY stateChanged)

public:
  // The instance QML gets (see qml_singleton.hpp).
  static TaskController *create(QQmlEngine *, QJSEngine *);
  static void setQmlInstance(TaskController *instance) { s_qmlInstance = instance; }

  TaskController(ArmController *arm, HandController *hand, QObject *parent = nullptr);

  QVariantList objects() const;
  QStringList objectTypes() const;
  int selectedId() const { return _selectedId; }
  void selectObject(int id);
  int heldId() const { return _heldId; }
  QVariantMap heldObject() const;

  bool busy() const { return !_steps.empty(); }
  bool reaching() const { return _following || _leftFollowing; }
  QString status() const { return _status; }
  QString clickMode() const { return _clickMode; }
  void setClickMode(const QString &mode);
  QString placeType() const { return _placeType; }
  void setPlaceType(const QString &type);
  double placeMassKg() const { return _placeMassKg; }
  void setPlaceMassKg(double kg);

  double waistYawDegrees() const;
  void setWaistYawDegrees(double degrees);
  double hipHeight() const { return _pose.hip_height; }
  void setHipHeight(double meters);
  double hipBendDegrees() const;
  void setHipBendDegrees(double degrees);
  double handRollDegrees() const;
  QVariantMap body() const;
  QVariantList tables() const;
  QStringList tableKinds() const;
  double gripperWidth() const { return _gripperWidth; }
  double grip() const { return _grip; }
  bool assist() const { return _assist; }
  void setAssist(bool on);
  int intentId() const { return _guess.id; }
  double intentConfidence() const { return _guess.confidence; }
  double assistStrength() const { return _assistStrength; }

  // Gives the robot a working left arm (not owned; both must outlive this):
  // a second ArmController for the same humanoid arm, mirrored onto the
  // left shoulder, and a HandController for its hand.
  void setLeftArm(ArmController *arm, HandController *hand);
  ArmController *leftArm() const { return _leftArm; }
  HandController *leftHand() const { return _leftHand; }
  int leftHeldId() const { return _leftHeldId; }
  QVariantMap leftHeldObject() const;
  double leftHandRollDegrees() const;
  double leftGrip() const { return _leftGrip; }

  // Actions. Slots, so QML calls them like functions
  // (TaskController.pickUp(id)) and C++ can connect signals to them.
public slots:
  // Sets the goal for the arm's end-effector point to the world point
  // (x, y, z): plan_reach finds the pose (changing as little as possible
  // from the previous goal), and the body then moves there at the speeds
  // its joints can manage - every turn, crouch and lean is visible, no
  // jumps. It's what dragging the hand calls on every mouse move. Returns
  // false if the point is out of reach (the hand then goes to the closest
  // point it can get to). Ignored while a pick / put-down sequence runs.
  bool reachTo(double x, double y, double z);
  // The same, with the hand also pointing `handAngleRadians` up from level
  // in the arm's plane (as far as the wrist goes there) - the webcam copies
  // the tilt of a person's hand with it.
  bool reachToPointing(double x, double y, double z, double handAngleRadians);
  // Turns the humanoid's hand about its own axis toward `radians` (see
  // handRollDegrees; limited to +-90 degrees), at the speed a forearm
  // twists - the webcam copies the turn of a person's palm with it. Ignored
  // for the warehouse gripper and during a pick / put-down, which turn the
  // hand back to neutral.
  void setHandRoll(double radians);
  // reachTo / reachToPointing for live control (the webcam): the same,
  // with the assist's help (see the assist property) - pass a NaN
  // handAngleRadians to leave the hand's direction free. Ignored while
  // the assist finishes a grab. The time since the last call is measured
  // here; after a long gap (the person was out of view) the guessing
  // starts afresh.
  bool guidedReach(double x, double y, double z, double handAngleRadians);

  // ---- the left arm (see setLeftArm; all ignored without one, and during
  // a pick / put-down) ----
  //
  // Live control of the left hand, like guidedReach (NaN handAngleRadians
  // = free). The two arms share one body, so one of them steers it: the
  // one whose hand moved (3 cm or more) most recently - the right, until
  // the left has moved. That arm turns, lifts and leans the body to get
  // where it's going; the other reaches with its own joints alone, toward
  // the part of its point in its plane. Returns false if it can't reach
  // the point.
  bool reachLeft(double x, double y, double z, double handAngleRadians);
  // The left hand's grip, 0 open .. 1 closed, like setGrip: closing grabs
  // the object the hand is around - or, within 20 cm of one, moves onto
  // it (body and all) and grabs on arrival; opening lets go onto what's
  // below.
  void setLeftGrip(double amount);
  // The left hand's twist, like setHandRoll (+ palm up).
  void setLeftHandRoll(double radians);
  // Stops the left arm where it is.
  void cancelLeftReach();
  // Stops moving toward the reachTo() goal, wherever the body is now.
  void cancelReach();
  // The mouse's grab: holding nothing, grabs the object the hand is
  // around - or, within 20 cm of one, moves the hand onto it and grabs on
  // arrival; holding something, lets go of it onto what's below. False
  // (and `status` says why) if nothing happened.
  bool grabOrRelease();

  // Shows a message in the status line - for events from outside the
  // controller (a webcam connecting...).
  void notify(const QString &message) { setStatus(message); }

  // Live hand control (webcam): 0 = open .. 1 = closed. Closes the
  // humanoid's fingers or the warehouse gripper's jaws right away; while
  // holding something they stay wrapped around it. Closing (to 0.7 or more)
  // grabs the object the hand is around; opening (to 0.3 or less) sets it
  // down on the surface right below the hand. Ignored during a
  // pick / put-down.
  void setGrip(double amount);

  // Switches the arm between the humanoid (shoulder / elbow / wrist +
  // hand) and an industrial warehouse arm (+ parallel gripper). Not while
  // holding something or mid-sequence.
  bool useHumanoidArm();
  bool useIndustrialArm();

  // Adds an object standing at (x, z) on whatever surface is there; its
  // mass defaults to the type's typical mass when massKg <= 0. Returns the
  // new id, or -1 (with `status` explaining) if it doesn't fit there.
  int addObject(const QString &type, double massKg, double x, double z);
  bool moveObject(int id, double x, double z);
  void setObjectMass(int id, double kg);
  void removeObject(int id);
  // Tables (and coffee tables, counters) centered at (x, z). Adding or
  // moving one fails (-1 / false, `status` says why) if it would overlap
  // the robot's base, another table or an object on the floor. Whatever
  // stands on a table moves with it; removing a table sets its objects
  // down on the floor where they were.
  int addTable(const QString &kind, double x, double z);
  bool moveTable(int id, double x, double z);
  void removeTable(int id);

  // Selects the object whose footprint contains (x, z), if any.
  bool selectAt(double x, double z);

  // Plans and starts the pick / put-down sequence. False (and `status`
  // says why) if it can't be done - out of reach, too heavy, no room...
  bool pickUp(int id);
  bool putDown(double x, double z);
  // Abandons the running sequence where it is.
  void stop();

  // One click in the 3D view at (x, z) on a surface, interpreted by
  // clickMode (see the property).
  void handleClick(double x, double z);

public:
  // guidedReach with the time since the previous call given, not
  // measured - for tests.
  bool guidedReach(double x, double y, double z, double handAngleRadians, double seconds);
  // reachLeft with whether the left arm steers the body given - for tests.
  bool reachLeft(double x, double y, double z, double handAngleRadians, bool leads);

  // A query, not an action - stays Q_INVOKABLE (callable from QML, but not
  // something to connect a signal to).
  // Height of the surface an object of footprint radius `footprint` would
  // stand on at (x, z): a table top, the floor, or -1 if it would hang
  // over a table's edge or overlap the robot's base.
  Q_INVOKABLE double supportHeightAt(double x, double z, double footprint) const;

  // Moves every running animation forward by `seconds`. Called by the
  // internal 60 Hz timer; public so tests can step time deterministically.
  void advance(double seconds);

signals:
  void objectsChanged();
  void stateChanged();
  void bodyChanged();
  void intentChanged();
  void leftArmChanged();
  void tablesChanged();

private:
  struct SceneObject {
    int id = 0;
    QString type;
    double mass = 0.0;
    Eigen::Vector3d base = Eigen::Vector3d::Zero(); // bottom center, world
    // How it's being held (while held): "hand", "top" (gripper from above)
    // or "side" (gripper level), and at what height above its base.
    QString grasp_mode;
    double grasp_height = 0.0;
  };

  // An axis-aligned table: center (x, z), size along X (width) and Z
  // (depth), top at `height`.
  struct Table {
    int id = 0;
    QString kind;
    double x = 0.0;
    double z = 0.0;
    double width = 0.5;
    double depth = 0.45;
    double height = 0.72;
    double x0() const { return x - width / 2.0; }
    double x1() const { return x + width / 2.0; }
    double z0() const { return z - depth / 2.0; }
    double z1() const { return z + depth / 2.0; }
  };

  // One way of grasping an object: the planner target, which mode it is,
  // and where on the object the grip is.
  struct GraspOption {
    GraspTarget target;
    QString mode;
    double height = 0.0;
  };

  // Size/shape facts a planner and a renderer both need.
  struct Shape {
    double grip_radius = 0.03; // radius of the part the fingers close around
    double footprint = 0.03;   // radius of what it stands on / occupies
    double height = 0.1;
    double grip_height = 0.05; // grip center above the base
    bool upright = true;       // must stay upright in the hand
  };

  struct Step {
    BodyPose target;
    double duration = 1.0;
    std::function<void()> on_arrive;
  };

  // One hand's live target, for which arm steers the body (reachLeft):
  // where it was, and when it last moved 3 cm or more.
  struct Mover {
    Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
    bool anchored = false;
    QElapsedTimer moved;
  };

  static double defaultMass(const QString &type);
  static Shape shapeOf(const QString &type, double mass);
  // Hand-frame position of a held object's grip center, and how far the
  // fingers close for it (see Hand3D.qml's frame: +X along the fingers,
  // +Y out of the palm).
  static double gripForward(double grip_radius);
  static double gripPalmar(double grip_radius);
  static double gripClosure(double grip_radius);

  Table *findTable(int id);
  // Why a table of this size can't stand at (x, z) - empty if it can.
  // `ignore_id` is the table being moved (its own objects ride along).
  QString tableBlockedBy(const Table &table, int ignore_id) const;
  bool isOn(const SceneObject &object, const Table &table) const;
  SceneObject *findObject(int id);
  const SceneObject *findObject(int id) const;
  bool spotIsFree(double x, double z, double footprint, int ignore_id) const;
  QVariantMap toVariant(const SceneObject &object) const;

  bool industrial() const;
  // Held by either hand.
  bool isHeld(int id) const { return id != -1 && (id == _heldId || id == _leftHeldId); }
  // Where an object of this grip radius would sit in the closed hand (or
  // between the gripper's jaws), in the world, for the current pose.
  Eigen::Vector3d handGripPoint(double grip_radius) const;
  // setGrip's grab and release; false (with a status, if `complain`) if
  // nothing happened.
  bool grabNearHand(bool complain = true);
  bool releaseUnderHand();
  // The ways this arm can grasp `object` standing with its base at `base`,
  // most preferred first (the hand: from the side; a gripper: from above,
  // then from the side).
  std::vector<GraspOption> graspOptions(const SceneObject &object, const Eigen::Vector3d &base) const;
  // The same grasp, `mode` and height fixed (for putting down what's held).
  GraspOption graspOption(const SceneObject &object, const Eigen::Vector3d &base, const QString &mode,
                          double height) const;
  // A pose `distance` back along the approach direction of `option` - the
  // hand slides in palm-first, the gripper comes down from above or in
  // along its own fingers.
  BodyPose approachPose(const GraspOption &option, const PickPlan &at, double distance) const;
  // Standing pose holding the hand / gripper in front of the body.
  BodyPose carryPose(const GraspOption &option, const PickPlan &fallback) const;
  BodyPose homePose(double yaw) const;
  // The current body pose with the arm's live joint values.
  BodyPose currentPose() const;
  // The same pose with the waist turned so the hand sits `clearance`
  // further from `grip_point` (on the back-of-hand side) - where the hand
  // approaches from and retreats to, so it never sweeps through the object.
  BodyPose sideOffPose(BodyPose pose, const Eigen::Vector3d &grip_point, double palmar_offset, double clearance) const;
  double stepDuration(const BodyPose &from, const BodyPose &to) const;
  void applyPose(const BodyPose &pose);
  bool reachToward(const Eigen::Vector3d &point, const double *hand_angle);
  // The arm's end-effector point in the world, for the current pose.
  Eigen::Vector3d endEffectorPoint() const;
  // Updates the guess from a live end-effector target `point` and pulls
  // it (and `hand_angle`, if given) toward the guessed object's grasp.
  Eigen::Vector3d assisted(const Eigen::Vector3d &point, double *hand_angle, double seconds);
  void forgetIntent();
  // Closing the hand near the guessed object, but not around it: plans
  // the grasp and moves there, grabbing on arrival. False if there's no
  // sure enough guess close enough, or no grasp the arm can make.
  bool finishGrab();
  // Plans the grasp of `object` and moves the hand there, grabbing it on
  // arrival; false if the arm can't make it.
  bool moveInToGrab(const SceneObject &object);
  // The object (not held) nearest where it would sit in the hand, within
  // `reach` m; nullptr if none.
  const SceneObject *nearestToHand(double reach) const;
  // For mouse drags: the guess (intentId) becomes the object in reach of
  // where the hand is going, so the 3D view rings it.
  void showWhatsInReach(const Eigen::Vector3d &point);
  // One time step of moving toward _goal (see reachTo).
  void followGoal(double seconds);
  void startTicker();
  // Stops the timer once nothing moves any more (no sequence, no goal for
  // either arm).
  void stopTickerIfIdle();

  // The left arm's side of things: its pose (the shared body + its own
  // joints and roll), where an object of this grip radius sits in its
  // hand, one time step toward its goal, and its grab / release.
  BodyPose leftPose() const;
  Eigen::Vector3d leftGripPoint(double grip_radius) const;
  void followLeft(double seconds);
  const SceneObject *nearestToLeftHand(double reach) const;
  bool grabNearLeftHand();
  bool releaseUnderLeftHand();
  bool leftMoveInToGrab(const SceneObject &object);
  // A grab the left hand was moving in for, once both it and the body
  // have arrived.
  void finishLeftGrabIfArrived();
  // Notes a live target for one hand; leftLeads: the left hand moved last.
  static void noteMove(Mover &mover, const Eigen::Vector3d &target);
  bool leftLeads() const;
  // The right arm reaching with its own joints alone (the left steers).
  bool reachWithArmOnly(const Eigen::Vector3d &point, const double *hand_angle);
  void setStatus(const QString &status);
  void queue(const BodyPose &target, double duration, std::function<void()> on_arrive = {});

  ArmController *_arm;
  HandController *_hand;
  BodyGeometry _geometry;

  std::vector<Table> _tables;
  int _nextTableId = 1;
  std::vector<SceneObject> _objects;
  int _nextId = 1;
  int _selectedId = -1;
  int _heldId = -1;

  BodyPose _pose; // yaw / hip height / bend; the arm angles live in ArmController
  std::deque<Step> _steps;
  BodyPose _stepStart;
  double _stepElapsed = 0.0;
  bool _stepStarted = false;
  // reachTo()'s goal, while the body is still on its way there.
  BodyPose _goal;
  bool _following = false;
  // Whether live input last closed the hand (past the grab threshold) -
  // grab and release fire when this flips, not on every update.
  bool _gripClosed = false;
  // The assist (see guidedReach): its guess, the current pull, and the
  // object it is moving the hand onto to grab (-1 = none).
  bool _assist = true;
  IntentEstimator _intent;
  Intent _guess;
  double _assistStrength = 0.0;
  int _finishingId = -1;
  QElapsedTimer _liveClock;

  // The left arm (see setLeftArm): its goal (joint values), roll, what it
  // holds, its grip, the object it's moving in to grab, and when the right
  // arm was last steered live (it then steers the body).
  ArmController *_leftArm = nullptr;
  HandController *_leftHand = nullptr;
  std::vector<double> _leftGoalArm;
  bool _leftFollowing = false;
  double _leftRoll = 0.0;
  double _leftGoalRoll = 0.0;
  int _leftHeldId = -1;
  double _leftGrip = 0.0;
  bool _leftGripClosed = false;
  int _leftFinishingId = -1;
  // Which arm steers the body (see reachLeft).
  Mover _rightMover;
  Mover _leftMover;
  QTimer _ticker;
  QElapsedTimer _tickClock;

  QString _status;
  QString _clickMode;
  QString _placeType = QStringLiteral("bottle");
  double _gripperWidth = 0.11;
  double _grip = 0.0;
  double _placeMassKg = 1.0;

  static constexpr double kBaseRadius = 0.32; // the robot's mobile base

  static inline TaskController *s_qmlInstance = nullptr;
};

} // namespace robotarm
