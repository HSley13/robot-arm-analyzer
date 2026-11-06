#pragma once

#include <deque>
#include <functional>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QElapsedTimer>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>

#include "robotarm/pick_planner.hpp"

namespace robotarm {

class ArmController;
class HandController;

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
  Q_PROPERTY(QVariantList objects READ objects NOTIFY objectsChanged)
  Q_PROPERTY(QStringList objectTypes READ objectTypes CONSTANT)
  Q_PROPERTY(int selectedId READ selectedId WRITE selectObject NOTIFY objectsChanged)
  Q_PROPERTY(int heldId READ heldId NOTIFY objectsChanged)
  Q_PROPERTY(QVariantMap heldObject READ heldObject NOTIFY objectsChanged)

  Q_PROPERTY(bool busy READ busy NOTIFY stateChanged)
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
  // Body dimensions and ranges for the 3D model and the sliders.
  Q_PROPERTY(QVariantMap body READ body CONSTANT)
  // The table: {x0, x1, z0, z1, height}.
  Q_PROPERTY(QVariantMap table READ table CONSTANT)

public:
  TaskController(ArmController *arm, HandController *hand, QObject *parent = nullptr);

  QVariantList objects() const;
  QStringList objectTypes() const;
  int selectedId() const { return _selectedId; }
  void selectObject(int id);
  int heldId() const { return _heldId; }
  QVariantMap heldObject() const;

  bool busy() const { return !_steps.empty(); }
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
  QVariantMap body() const;
  QVariantMap table() const;

  // Adds an object standing at (x, z) on whatever surface is there; its
  // mass defaults to the type's typical mass when massKg <= 0. Returns the
  // new id, or -1 (with `status` explaining) if it doesn't fit there.
  Q_INVOKABLE int addObject(const QString &type, double massKg, double x, double z);
  Q_INVOKABLE bool moveObject(int id, double x, double z);
  Q_INVOKABLE void setObjectMass(int id, double kg);
  Q_INVOKABLE void removeObject(int id);
  // Selects the object whose footprint contains (x, z), if any.
  Q_INVOKABLE bool selectAt(double x, double z);

  // Plans and starts the pick / put-down sequence. False (and `status`
  // says why) if it can't be done - out of reach, too heavy, no room...
  Q_INVOKABLE bool pickUp(int id);
  Q_INVOKABLE bool putDown(double x, double z);
  // Abandons the running sequence where it is.
  Q_INVOKABLE void stop();

  // One click in the 3D view at (x, z) on a surface, interpreted by
  // clickMode (see the property).
  Q_INVOKABLE void handleClick(double x, double z);

  // Height of the surface an object of footprint radius `footprint` would
  // stand on at (x, z): the table top, the floor, or -1 if it would hang
  // over the table's edge or overlap the robot's base.
  Q_INVOKABLE double supportHeightAt(double x, double z, double footprint) const;

  // Moves every running animation forward by `seconds`. Called by the
  // internal 60 Hz timer; public so tests can step time deterministically.
  void advance(double seconds);

signals:
  void objectsChanged();
  void stateChanged();
  void bodyChanged();

private:
  struct SceneObject {
    int id = 0;
    QString type;
    double mass = 0.0;
    Eigen::Vector3d base = Eigen::Vector3d::Zero(); // bottom center, world
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

  static double defaultMass(const QString &type);
  static Shape shapeOf(const QString &type, double mass);
  // Hand-frame position of a held object's grip center, and how far the
  // fingers close for it (see Hand3D.qml's frame: +X along the fingers,
  // +Y out of the palm).
  static double gripForward(double grip_radius);
  static double gripPalmar(double grip_radius);
  static double gripClosure(double grip_radius);

  SceneObject *findObject(int id);
  const SceneObject *findObject(int id) const;
  bool spotIsFree(double x, double z, double footprint, int ignore_id) const;
  QVariantMap toVariant(const SceneObject &object) const;

  GraspTarget graspTargetFor(const SceneObject &object, const Eigen::Vector3d &grip_point) const;
  // The current body pose with the arm's live joint values.
  BodyPose currentPose() const;
  // The same pose with the waist turned so the hand sits `clearance`
  // further from `grip_point` (on the back-of-hand side) - where the hand
  // approaches from and retreats to, so it never sweeps through the object.
  BodyPose sideOffPose(BodyPose pose, const Eigen::Vector3d &grip_point, double palmar_offset, double clearance) const;
  double stepDuration(const BodyPose &from, const BodyPose &to) const;
  void applyPose(const BodyPose &pose);
  void setStatus(const QString &status);
  void queue(const BodyPose &target, double duration, std::function<void()> on_arrive = {});

  ArmController *_arm;
  HandController *_hand;
  BodyGeometry _geometry;

  std::vector<SceneObject> _objects;
  int _nextId = 1;
  int _selectedId = -1;
  int _heldId = -1;

  BodyPose _pose; // yaw / hip height / bend; the arm angles live in ArmController
  std::deque<Step> _steps;
  BodyPose _stepStart;
  double _stepElapsed = 0.0;
  bool _stepStarted = false;
  QTimer _ticker;
  QElapsedTimer _tickClock;

  QString _status;
  QString _clickMode;
  QString _placeType = QStringLiteral("bottle");
  double _placeMassKg = 1.0;

  // The table, in front and to the robot's left.
  static constexpr double kTableX0 = 0.25;
  static constexpr double kTableX1 = 0.75;
  static constexpr double kTableZ0 = -0.80;
  static constexpr double kTableZ1 = -0.35;
  static constexpr double kTableHeight = 0.72;
  static constexpr double kBaseRadius = 0.32; // the robot's mobile base
};

} // namespace robotarm
