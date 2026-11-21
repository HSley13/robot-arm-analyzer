#include "task_controller.hpp"

#include "qml_singleton.hpp"

#include <algorithm>
#include <cmath>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

TaskController *TaskController::create(QQmlEngine *, QJSEngine *) { return existingQmlSingleton(s_qmlInstance); }

namespace {

constexpr double kApproachClearance = 0.10; // m, hand starts/ends this far from the object, palm toward it
constexpr double kLiftHeight = 0.12;        // m, how far a picked object is raised (and lowered onto a spot)
constexpr double kTickSeconds = 1.0 / 60.0;
constexpr double kStandingHipHeight = 0.80; // m, the lift's normal standing height
constexpr double kGripperReach = 0.075;     // m, gripper finger center past the arm's flange
constexpr double kGripperOpen = 0.11;       // m, gripper fingers wide open
constexpr double kGripperShut = 0.01;       // m, gripper fingers closed on nothing

// Live grip (setGrip): closing past kGrabAt grabs, opening below
// kReleaseAt lets go. The gap keeps a half-closed, jittery hand from
// flickering between the two.
constexpr double kGrabAt = 0.7;
constexpr double kReleaseAt = 0.3;
constexpr double kGrabReach = 0.08; // m, hand to object's grip center, to grab it

// The assist (guidedReach): closing the hand within kFinishReach of the
// guessed object's grip center, guessed at least kFinishConfidence sure,
// moves the hand onto it and grabs. Live updates further apart than
// kLiveGap start the guessing afresh.
constexpr double kFinishReach = 0.20;     // m
constexpr double kFinishConfidence = 0.45;
constexpr double kLiveGap = 0.5;          // s
constexpr double kTypicalGripRadius = 0.03; // m, for where "the hand" is while guessing
// A hand whose live target moves this far is "moving" - the last one
// moving steers the body (reachLeft).
constexpr double kMoved = 0.03; // m

// Following a reachTo() goal: every degree of freedom closes the gap
// exponentially (fast for small moves, so dragging feels direct), no
// faster than its top speed, and speeding up or slowing down no faster
// than its top acceleration - all together, so a big move is one
// coordinated motion whose parts finish at the same time.
constexpr double kFollowRate = 10.0;       // 1/s
constexpr double kMaxYawSpeed = 1.8;       // rad/s (~100 deg/s)
constexpr double kMaxLiftSpeed = 0.45;     // m/s
constexpr double kMaxBendSpeed = 1.2;      // rad/s (~70 deg/s)
constexpr double kMaxJointSpeed = 2.6;     // rad/s (~150 deg/s)
constexpr double kMaxRollSpeed = 3.5;      // rad/s (~200 deg/s), a forearm twisting
constexpr double kMaxYawAccel = 6.0;       // rad/s^2
constexpr double kMaxLiftAccel = 2.0;      // m/s^2
constexpr double kMaxBendAccel = 5.0;      // rad/s^2
constexpr double kMaxJointAccel = 10.0;    // rad/s^2
constexpr double kMaxRollAccel = 20.0;     // rad/s^2
constexpr double kMaxHandRoll = kPi / 2.0; // rad, palm straight up or down
constexpr double kArrived = 1e-3;          // rad or m left to go

double wrap(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// One time step of following: what each degree of freedom moves by, and
// its new speed and its acceleration over the step. They all head for
// their gaps at one common rate (so they arrive together): kFollowRate,
// lowered until none goes faster than its top speed, nor faster than it
// could still stop from in the distance left (braking at its top
// acceleration in steps of `seconds`: v^2 / 2a + v * seconds / 2 <= gap).
// The speeds then change toward that, together, by no more than any top
// acceleration allows.
struct FollowStep {
  std::vector<double> move;
  std::vector<double> speed;
  std::vector<double> acceleration;
};

FollowStep followStep(const std::vector<double> &gap, const std::vector<double> &speed,
                      const std::vector<double> &top_speed, const std::vector<double> &top_accel, double seconds) {
  double rate = kFollowRate;
  for (size_t i = 0; i < gap.size(); i++) {
    const double distance = std::abs(gap[i]);
    if (distance > 1e-12) {
      const double half_step = top_accel[i] * seconds / 2.0;
      const double stoppable = std::sqrt(half_step * half_step + 2.0 * top_accel[i] * distance) - half_step;
      rate = std::min({rate, top_speed[i] / distance, stoppable / distance});
    }
  }
  double share = 1.0;
  for (size_t i = 0; i < gap.size(); i++) {
    const double change = std::abs(rate * gap[i] - speed[i]);
    if (change > top_accel[i] * seconds) {
      share = std::min(share, top_accel[i] * seconds / change);
    }
  }

  FollowStep step;
  for (size_t i = 0; i < gap.size(); i++) {
    const double v = speed[i] + share * (rate * gap[i] - speed[i]);
    double move = v * seconds;
    if (move * gap[i] > 0.0 && std::abs(move) > std::abs(gap[i])) {
      move = gap[i]; // never past the goal
    }
    step.move.push_back(move);
    step.speed.push_back(v);
    step.acceleration.push_back((v - speed[i]) / seconds);
  }
  return step;
}

QString labelOf(const QString &type) {
  if (type.isEmpty()) {
    return type;
  }
  return type.left(1).toUpper() + type.mid(1);
}

QString massText(double kg) { return QString::number(kg, 'f', kg < 10.0 ? 1 : 0) + " kg"; }

} // namespace

TaskController::TaskController(ArmController *arm, HandController *hand, QObject *parent)
    : QObject(parent), _arm(arm), _hand(hand) {
  _pose.yaw = 0.0;
  _pose.hip_height = kStandingHipHeight;
  _pose.hip_bend = 0.0;
  _arm->setBaseTiltDegrees(0.0);
  addTable(QStringLiteral("table"), 0.50, -0.575); // in front and to the robot's left
  _status.clear();

  // Advance by real elapsed time, so the motion keeps its speed even if
  // the window renders slowly; capped so a long stall doesn't teleport.
  _ticker.setInterval(static_cast<int>(kTickSeconds * 1000.0));
  connect(&_ticker, &QTimer::timeout, this, [this]() {
    const double elapsed = _tickClock.isValid() ? static_cast<double>(_tickClock.restart()) / 1000.0 : kTickSeconds;
    advance(std::min(elapsed, 0.25));
  });
}

// ---- object catalogue ------------------------------------------------------

QStringList TaskController::objectTypes() const {
  return {QStringLiteral("bottle"), QStringLiteral("can"),   QStringLiteral("mug"),
          QStringLiteral("box"),    QStringLiteral("book"),  QStringLiteral("ball"),
          QStringLiteral("cone"),   QStringLiteral("plant"), QStringLiteral("dumbbell"),
          QStringLiteral("kettlebell")};
}

double TaskController::defaultMass(const QString &type) {
  if (type == QStringLiteral("can")) {
    return 0.35;
  }
  if (type == QStringLiteral("box")) {
    return 1.2;
  }
  if (type == QStringLiteral("ball")) {
    return 0.45;
  }
  if (type == QStringLiteral("dumbbell") || type == QStringLiteral("kettlebell")) {
    return 8.0;
  }
  if (type == QStringLiteral("mug")) {
    return 0.35;
  }
  if (type == QStringLiteral("book")) {
    return 0.6;
  }
  if (type == QStringLiteral("cone")) {
    return 0.7;
  }
  if (type == QStringLiteral("plant")) {
    return 2.0;
  }
  return 1.0; // bottle
}

TaskController::Shape TaskController::shapeOf(const QString &type, double mass) {
  Shape shape;
  if (type == QStringLiteral("can")) {
    shape = {0.033, 0.034, 0.123, 0.062, true};
  } else if (type == QStringLiteral("box")) {
    // 6.4 cm square, so the fingers close around its sides like a radius
    // of 3.2 cm; its corners reach 4.5 cm out.
    shape = {0.032, 0.046, 0.16, 0.08, true};
  } else if (type == QStringLiteral("ball")) {
    shape = {0.05, 0.05, 0.10, 0.05, false};
  } else if (type == QStringLiteral("dumbbell")) {
    // Standing on one end: plates sized by mass (same formulas Hand3D.qml
    // uses for the payload dumbbell), grasped by the handle between them.
    const double plate_thickness = std::min(0.07, 0.010 + 0.003 * mass);
    const double plate_radius = std::min(0.10, 0.030 + 0.0095 * std::sqrt(std::max(0.0, mass)));
    shape = {0.0125, plate_radius, 2.0 * plate_thickness + 0.132, plate_thickness + 0.066, true};
  } else if (type == QStringLiteral("mug")) {
    // 8 cm across; the handle sticks out to 5 cm from the center.
    shape = {0.040, 0.050, 0.10, 0.05, true};
  } else if (type == QStringLiteral("book")) {
    // Standing on its bottom edge, 3.5 cm thick, 15 cm wide, 22 cm tall -
    // gripped across its thickness, two thirds of the way up.
    shape = {0.0175, 0.08, 0.22, 0.15, true};
  } else if (type == QStringLiteral("cone")) {
    // A traffic cone on an 18 cm square base, held around its upper part.
    shape = {0.028, 0.09, 0.32, 0.22, true};
  } else if (type == QStringLiteral("plant")) {
    // A potted plant, held around the pot; the leaves reach 38 cm.
    shape = {0.065, 0.075, 0.38, 0.08, true};
  } else if (type == QStringLiteral("kettlebell")) {
    // Iron ball sized by mass (cast iron, ~7 g/cm^3) with a handle on top,
    // held by the handle.
    const double radius = std::clamp(std::cbrt(3.0 * mass / (4.0 * kPi * 7000.0)) * 1.15, 0.045, 0.13);
    shape = {0.016, radius, 2.0 * radius + 0.075, 2.0 * radius + 0.055, true};
  } else {
    shape = {0.033, 0.034, 0.23, 0.10, true}; // bottle
  }
  return shape;
}

// Fitted to the fingers' own forward kinematics at the closure each size
// gets (gripClosure): the object's center sits just past the knuckles,
// about one radius off the palm.
double TaskController::gripForward(double /*grip_radius*/) { return 0.094; }
double TaskController::gripPalmar(double grip_radius) { return 0.024 + 0.8 * grip_radius; }
double TaskController::gripClosure(double grip_radius) {
  // A 2.5 cm handle needs the fingers ~70% closed; a 10 cm ball ~33%.
  return std::clamp(0.7 - (grip_radius - 0.014) * 10.5, 0.25, 0.75);
}

// ---- objects -----------------------------------------------------------------

TaskController::SceneObject *TaskController::findObject(int id) {
  for (SceneObject &object : _objects) {
    if (object.id == id) {
      return &object;
    }
  }
  return nullptr;
}

const TaskController::SceneObject *TaskController::findObject(int id) const {
  for (const SceneObject &object : _objects) {
    if (object.id == id) {
      return &object;
    }
  }
  return nullptr;
}

QVariantMap TaskController::toVariant(const SceneObject &object) const {
  const Shape shape = shapeOf(object.type, object.mass);
  QVariantMap map;
  map["id"] = object.id;
  map["type"] = object.type;
  map["label"] = labelOf(object.type);
  map["mass"] = object.mass;
  map["x"] = object.base.x();
  map["y"] = object.base.y();
  map["z"] = object.base.z();
  map["gripRadius"] = shape.grip_radius;
  map["footprint"] = shape.footprint;
  map["height"] = shape.height;
  map["gripHeight"] = shape.grip_height;
  map["held"] = isHeld(object.id);
  map["selected"] = object.id == _selectedId;
  // Where it sits in the hand (Hand3D's frame) while held.
  map["gripForward"] = gripForward(shape.grip_radius);
  map["gripPalmar"] = gripPalmar(shape.grip_radius);
  map["graspMode"] = object.grasp_mode;
  map["graspHeight"] = object.grasp_height;
  map["gripperReach"] = kGripperReach;
  return map;
}

QVariantList TaskController::objects() const {
  QVariantList list;
  for (const SceneObject &object : _objects) {
    list.append(toVariant(object));
  }
  return list;
}

QVariantMap TaskController::heldObject() const {
  const SceneObject *object = findObject(_heldId);
  return object == nullptr ? QVariantMap() : toVariant(*object);
}

void TaskController::selectObject(int id) {
  if (id != -1 && findObject(id) == nullptr) {
    return;
  }
  _selectedId = id;
  emit objectsChanged();
}

double TaskController::supportHeightAt(double x, double z, double footprint) const {
  if (std::hypot(x, z) < kBaseRadius + footprint) {
    return -1.0; // on the robot's own base
  }
  for (const Table &table : _tables) {
    const bool fully_on = x >= table.x0() + footprint && x <= table.x1() - footprint &&
                          z >= table.z0() + footprint && z <= table.z1() - footprint;
    if (fully_on) {
      return table.height;
    }
    const double dx = std::max({table.x0() - x, 0.0, x - table.x1()});
    const double dz = std::max({table.z0() - z, 0.0, z - table.z1()});
    if (std::hypot(dx, dz) <= footprint) {
      return -1.0; // straddling this table's edge (or under it)
    }
  }
  return 0.0; // clear of every table: the floor
}

// ---- tables ----------------------------------------------------------------

QStringList TaskController::tableKinds() const {
  return {QStringLiteral("table"), QStringLiteral("coffee table"), QStringLiteral("counter")};
}

TaskController::Table *TaskController::findTable(int id) {
  for (Table &table : _tables) {
    if (table.id == id) {
      return &table;
    }
  }
  return nullptr;
}

bool TaskController::isOn(const SceneObject &object, const Table &table) const {
  return !isHeld(object.id) && std::abs(object.base.y() - table.height) < 1e-9 && object.base.x() >= table.x0() &&
         object.base.x() <= table.x1() && object.base.z() >= table.z0() && object.base.z() <= table.z1();
}

QString TaskController::tableBlockedBy(const Table &table, int ignore_id) const {
  constexpr double kGap = 0.02; // m, room to walk a hand between things
  // The robot's base: closest point of the rectangle to the waist axis.
  const double nx = std::clamp(0.0, table.x0(), table.x1());
  const double nz = std::clamp(0.0, table.z0(), table.z1());
  if (std::hypot(nx, nz) < kBaseRadius + kGap) {
    return QStringLiteral("too close to the robot");
  }
  for (const Table &other : _tables) {
    if (other.id == ignore_id) {
      continue;
    }
    if (table.x0() < other.x1() + kGap && other.x0() < table.x1() + kGap && table.z0() < other.z1() + kGap &&
        other.z0() < table.z1() + kGap) {
      return QStringLiteral("another %1 is in the way").arg(other.kind);
    }
  }
  for (const SceneObject &object : _objects) {
    if (isHeld(object.id) || object.base.y() > 0.0) {
      continue; // carried, or standing on a table (a moved table takes its own along)
    }
    const double footprint = shapeOf(object.type, object.mass).footprint;
    const double dx = std::max({table.x0() - object.base.x(), 0.0, object.base.x() - table.x1()});
    const double dz = std::max({table.z0() - object.base.z(), 0.0, object.base.z() - table.z1()});
    if (std::hypot(dx, dz) <= footprint) {
      return QStringLiteral("the %1 on the floor is in the way").arg(object.type);
    }
  }
  return {};
}

QVariantList TaskController::tables() const {
  QVariantList list;
  for (const Table &table : _tables) {
    QVariantMap map;
    map["id"] = table.id;
    map["kind"] = table.kind;
    map["label"] = labelOf(table.kind);
    map["x"] = table.x;
    map["z"] = table.z;
    map["width"] = table.width;
    map["depth"] = table.depth;
    map["height"] = table.height;
    map["x0"] = table.x0();
    map["x1"] = table.x1();
    map["z0"] = table.z0();
    map["z1"] = table.z1();
    list.append(map);
  }
  return list;
}

int TaskController::addTable(const QString &kind, double x, double z) {
  Table table;
  table.kind = kind;
  table.x = x;
  table.z = z;
  if (kind == QStringLiteral("table")) {
    table.width = 0.50;
    table.depth = 0.45;
    table.height = 0.72;
  } else if (kind == QStringLiteral("coffee table")) {
    table.width = 0.70;
    table.depth = 0.42;
    table.height = 0.42;
  } else if (kind == QStringLiteral("counter")) {
    table.width = 0.42;
    table.depth = 0.60;
    table.height = 0.92;
  } else {
    setStatus(QStringLiteral("Unknown kind of table '%1'").arg(kind));
    return -1;
  }
  const QString blocked = tableBlockedBy(table, -1);
  if (!blocked.isEmpty()) {
    setStatus(QStringLiteral("No room for a %1 there - %2").arg(kind, blocked));
    return -1;
  }
  table.id = _nextTableId;
  _nextTableId++;
  _tables.push_back(table);
  setStatus(QStringLiteral("Added a %1").arg(kind));
  emit tablesChanged();
  return table.id;
}

bool TaskController::moveTable(int id, double x, double z) {
  Table *table = findTable(id);
  if (table == nullptr) {
    return false;
  }
  Table moved = *table;
  moved.x = x;
  moved.z = z;
  if (!tableBlockedBy(moved, id).isEmpty()) {
    return false; // dragging into something: just stay put
  }
  // What stands on it rides along.
  const Eigen::Vector3d shift(x - table->x, 0.0, z - table->z);
  bool carried = false;
  for (SceneObject &object : _objects) {
    if (isOn(object, *table)) {
      object.base += shift;
      carried = true;
    }
  }
  *table = moved;
  emit tablesChanged();
  if (carried) {
    emit objectsChanged();
  }
  return true;
}

void TaskController::removeTable(int id) {
  const Table *table = findTable(id);
  if (table == nullptr || busy()) {
    return;
  }
  const QString kind = table->kind;
  bool dropped = false;
  for (SceneObject &object : _objects) {
    if (isOn(object, *table)) {
      object.base.y() = 0.0; // set down on the floor right below
      dropped = true;
    }
  }
  _tables.erase(std::remove_if(_tables.begin(), _tables.end(), [id](const Table &t) { return t.id == id; }),
                _tables.end());
  setStatus(dropped ? QStringLiteral("Removed the %1 - what was on it is on the floor now").arg(kind)
                    : QStringLiteral("Removed the %1").arg(kind));
  emit tablesChanged();
  if (dropped) {
    emit objectsChanged();
  }
}

bool TaskController::spotIsFree(double x, double z, double footprint, int ignore_id) const {
  for (const SceneObject &other : _objects) {
    if (other.id == ignore_id || isHeld(other.id)) {
      continue;
    }
    const double distance = std::hypot(other.base.x() - x, other.base.z() - z);
    if (distance < footprint + shapeOf(other.type, other.mass).footprint + 0.01) {
      return false;
    }
  }
  return true;
}

int TaskController::addObject(const QString &type, double massKg, double x, double z) {
  if (!objectTypes().contains(type)) {
    setStatus(QStringLiteral("Unknown object type '%1'").arg(type));
    return -1;
  }
  const double mass = massKg > 0.0 ? massKg : defaultMass(type);
  const Shape shape = shapeOf(type, mass);
  const double support = supportHeightAt(x, z, shape.footprint);
  if (support < 0.0) {
    setStatus(QStringLiteral("No room for the %1 there (table edge or the robot's base)").arg(type));
    return -1;
  }
  if (!spotIsFree(x, z, shape.footprint, -1)) {
    setStatus(QStringLiteral("Another object is already there"));
    return -1;
  }

  SceneObject object;
  object.id = _nextId;
  _nextId++;
  object.type = type;
  object.mass = mass;
  object.base = Eigen::Vector3d(x, support, z);
  _objects.push_back(object);
  _selectedId = object.id;
  setStatus(QStringLiteral("Placed a %1 (%2) on the %3")
                .arg(type, massText(mass), support > 0.0 ? QStringLiteral("table") : QStringLiteral("floor")));
  emit objectsChanged();
  return object.id;
}

bool TaskController::moveObject(int id, double x, double z) {
  SceneObject *object = findObject(id);
  if (object == nullptr || isHeld(id)) {
    return false;
  }
  const Shape shape = shapeOf(object->type, object->mass);
  const double support = supportHeightAt(x, z, shape.footprint);
  if (support < 0.0 || !spotIsFree(x, z, shape.footprint, id)) {
    setStatus(QStringLiteral("Can't move the %1 there").arg(object->type));
    return false;
  }
  object->base = Eigen::Vector3d(x, support, z);
  emit objectsChanged();
  return true;
}

void TaskController::setObjectMass(int id, double kg) {
  SceneObject *object = findObject(id);
  if (object == nullptr || kg <= 0.0) {
    return;
  }
  object->mass = kg;
  if (id == _heldId) {
    _arm->setPayloadKg(kg); // the arm feels it immediately
  } else if (id == _leftHeldId) {
    _leftArm->setPayloadKg(kg);
  }
  emit objectsChanged();
}

void TaskController::removeObject(int id) {
  if (isHeld(id) || busy()) {
    return;
  }
  _objects.erase(std::remove_if(_objects.begin(), _objects.end(),
                                [id](const SceneObject &object) { return object.id == id; }),
                 _objects.end());
  if (_selectedId == id) {
    _selectedId = -1;
  }
  emit objectsChanged();
}

bool TaskController::selectAt(double x, double z) {
  for (const SceneObject &object : _objects) {
    if (isHeld(object.id)) {
      continue;
    }
    const double reach = shapeOf(object.type, object.mass).footprint + 0.02;
    if (std::hypot(object.base.x() - x, object.base.z() - z) <= reach) {
      selectObject(object.id);
      return true;
    }
  }
  selectObject(-1);
  return false;
}

// ---- body ------------------------------------------------------------------

double TaskController::waistYawDegrees() const { return to_degrees(_pose.yaw); }

double TaskController::handRollDegrees() const { return to_degrees(_pose.hand_roll); }
double TaskController::hipBendDegrees() const { return to_degrees(_pose.hip_bend); }

void TaskController::setWaistYawDegrees(double degrees) {
  if (busy()) {
    return;
  }
  cancelReach(); // a hand on the body overrides the hand goal
  BodyPose pose = currentPose();
  pose.yaw = to_radians(std::clamp(degrees, _geometry.min_yaw_degrees, _geometry.max_yaw_degrees));
  applyPose(pose);
}

void TaskController::setHipHeight(double meters) {
  if (busy()) {
    return;
  }
  cancelReach(); // a hand on the body overrides the hand goal
  BodyPose pose = currentPose();
  pose.hip_height = std::clamp(meters, _geometry.min_hip_height, _geometry.max_hip_height);
  applyPose(pose);
}

void TaskController::setHipBendDegrees(double degrees) {
  if (busy()) {
    return;
  }
  cancelReach(); // a hand on the body overrides the hand goal
  BodyPose pose = currentPose();
  pose.hip_bend = to_radians(std::clamp(degrees, 0.0, _geometry.max_hip_bend_degrees));
  applyPose(pose);
}

QVariantMap TaskController::body() const {
  QVariantMap map;
  map["shoulderLateral"] = _geometry.shoulder_lateral;
  map["hipToShoulder"] = _geometry.hip_to_shoulder;
  map["minHipHeight"] = _geometry.min_hip_height;
  map["maxHipHeight"] = _geometry.max_hip_height;
  map["maxHipBend"] = _geometry.max_hip_bend_degrees;
  map["minYaw"] = _geometry.min_yaw_degrees;
  map["maxYaw"] = _geometry.max_yaw_degrees;
  map["baseRadius"] = kBaseRadius;
  map["standingHipHeight"] = kStandingHipHeight;
  return map;
}

BodyPose TaskController::currentPose() const {
  BodyPose pose = _pose;
  pose.arm = _arm->jointAnglesRadians();
  return pose;
}

void TaskController::applyPose(const BodyPose &pose) {
  _pose.yaw = pose.yaw;
  _pose.hip_height = pose.hip_height;
  _pose.hip_bend = pose.hip_bend;
  _pose.hand_roll = industrial() ? 0.0 : pose.hand_roll;
  _arm->setBaseTiltDegrees(to_degrees(pose.hip_bend));
  if (_leftArm != nullptr) {
    _leftArm->setBaseTiltDegrees(to_degrees(pose.hip_bend)); // one torso, both arms lean with it
  }
  if (!pose.arm.empty()) {
    _arm->setJointAnglesRadians(pose.arm);
  }
  emit bodyChanged();
}

// ---- planning + sequences ------------------------------------------------

bool TaskController::industrial() const { return _arm->config().style == "industrial"; }

TaskController::GraspOption TaskController::graspOption(const SceneObject &object, const Eigen::Vector3d &base,
                                                        const QString &mode, double height) const {
  const Shape shape = shapeOf(object.type, object.mass);
  const ArmConfig &config = _arm->config();
  const double hand_link = config.joints.empty() ? 0.0 : config.joints.back().dh.a;

  GraspOption option;
  option.mode = mode;
  option.height = height;
  option.target.grip_point = base + Eigen::Vector3d(0.0, height, 0.0);
  option.target.mass = object.mass;
  option.target.keep_hand_level = shape.upright;
  if (mode == QStringLiteral("hand")) {
    option.target.palmar_offset = gripPalmar(shape.grip_radius);
    option.target.forward_offset = gripForward(shape.grip_radius) - hand_link;
  } else {
    // Parallel gripper: the fingers close across the arm's plane, so the
    // object sits centered in the plane, kGripperReach past the flange.
    option.target.palmar_offset = 0.0;
    option.target.forward_offset = kGripperReach;
    option.target.hand_angles = {mode == QStringLiteral("top") ? -kPi / 2.0 : 0.0};
    if (!shape.upright && mode == QStringLiteral("side")) {
      option.target.hand_angles.push_back(-kPi / 6.0);
      option.target.hand_angles.push_back(-kPi / 3.0);
    }
  }
  return option;
}

std::vector<TaskController::GraspOption> TaskController::graspOptions(const SceneObject &object,
                                                                      const Eigen::Vector3d &base) const {
  const Shape shape = shapeOf(object.type, object.mass);
  std::vector<GraspOption> options;
  if (!industrial()) {
    options.push_back(graspOption(object, base, QStringLiteral("hand"), shape.grip_height));
    return options;
  }
  // From above first (the warehouse way), gripping an upright object near
  // its top so the gripper's palm clears it - except a standing dumbbell,
  // whose top plate is in the way, and a plant (all leaves up there).
  if (object.type != QStringLiteral("dumbbell") && object.type != QStringLiteral("plant")) {
    const double top_height = shape.upright ? std::max(shape.grip_height, shape.height - 0.035) : shape.grip_height;
    options.push_back(graspOption(object, base, QStringLiteral("top"), top_height));
  }
  options.push_back(graspOption(object, base, QStringLiteral("side"), shape.grip_height));
  return options;
}

BodyPose TaskController::approachPose(const GraspOption &option, const PickPlan &at, double distance) const {
  if (option.mode == QStringLiteral("hand")) {
    return sideOffPose(at.pose, option.target.grip_point, option.target.palmar_offset, distance);
  }
  GraspTarget back = option.target;
  back.hand_angles = {at.hand_angle};
  if (option.mode == QStringLiteral("top")) {
    back.grip_point.y() += distance; // straight down onto it
  } else {
    // In along the gripper's own direction (level, facing the torso's way).
    const Eigen::Vector3d forward(std::cos(at.pose.yaw), 0.0, -std::sin(at.pose.yaw));
    back.grip_point -= distance * forward;
  }
  const PickPlan plan = plan_grasp(_arm->config(), _geometry, back, at.pose.hip_height);
  return plan.ok ? plan.pose : at.pose;
}

BodyPose TaskController::carryPose(const GraspOption &option, const PickPlan &fallback) const {
  // Held in front of the body at about waist height, standing upright.
  const double yaw = fallback.pose.yaw;
  const double lateral = _geometry.shoulder_lateral - option.target.palmar_offset;
  const double forward = 0.40;
  GraspTarget carry = option.target;
  carry.grip_point = Eigen::Vector3d(forward * std::cos(yaw) + lateral * std::sin(yaw), industrial() ? 1.00 : 0.95,
                                     -forward * std::sin(yaw) + lateral * std::cos(yaw));
  carry.hand_angles = {fallback.hand_angle};
  const PickPlan plan = plan_grasp(_arm->config(), _geometry, carry, kStandingHipHeight);
  return plan.ok ? plan.pose : fallback.pose;
}

BodyPose TaskController::homePose(double yaw) const {
  BodyPose home = currentPose();
  home.yaw = yaw;
  home.hand_roll = 0.0;
  home.hip_height = kStandingHipHeight;
  home.hip_bend = 0.0;
  if (!industrial() && home.arm.size() == 3) {
    home.arm = {to_radians(-70.0), to_radians(65.0), to_radians(5.0)}; // the humanoid's relaxed ready pose
    return home;
  }
  // A warehouse arm: tip tucked in front of the chest.
  const Eigen::Vector3d point(0.30 * std::cos(yaw) + _geometry.shoulder_lateral * std::sin(yaw), 1.05,
                              -0.30 * std::sin(yaw) + _geometry.shoulder_lateral * std::cos(yaw));
  const PickPlan plan = plan_reach(_arm->config(), _geometry, point, home);
  if (plan.ok) {
    home.arm = plan.pose.arm;
  }
  return home;
}

bool TaskController::reachTo(double x, double y, double z) {
  const bool reached = reachToward(Eigen::Vector3d(x, y, z), nullptr);
  showWhatsInReach(Eigen::Vector3d(x, y, z));
  return reached;
}

bool TaskController::reachToPointing(double x, double y, double z, double handAngleRadians) {
  return reachToward(Eigen::Vector3d(x, y, z), &handAngleRadians);
}

bool TaskController::guidedReach(double x, double y, double z, double handAngleRadians) {
  const double seconds = _liveClock.isValid() ? static_cast<double>(_liveClock.restart()) / 1000.0 : 0.0;
  if (!_liveClock.isValid()) {
    _liveClock.start();
  }
  return guidedReach(x, y, z, handAngleRadians, seconds);
}

bool TaskController::guidedReach(double x, double y, double z, double handAngleRadians, double seconds) {
  if (busy() || _finishingId != -1 || _leftFinishingId != -1) {
    return false;
  }
  noteMove(_rightMover, Eigen::Vector3d(x, y, z));
  if (seconds > kLiveGap) {
    forgetIntent();
    seconds = 0.0;
  }
  double hand_angle = handAngleRadians;
  const bool pointing = std::isfinite(hand_angle);
  const Eigen::Vector3d point = assisted(Eigen::Vector3d(x, y, z), pointing ? &hand_angle : nullptr, seconds);
  if (leftLeads()) {
    return reachWithArmOnly(point, pointing ? &hand_angle : nullptr);
  }
  return reachToward(point, pointing ? &hand_angle : nullptr);
}

bool TaskController::reachWithArmOnly(const Eigen::Vector3d &point, const double *hand_angle) {
  const BodyPose from = _following ? _goal : currentPose();
  const PickPlan plan = plan_arm_reach(_arm->config(), _geometry, point, from, hand_angle);
  if (plan.pose.arm.size() != _arm->config().joints.size()) {
    return false;
  }
  _goal = from;
  _goal.arm = plan.pose.arm;
  if (!_following) {
    _following = true;
    emit stateChanged();
  }
  startTicker();
  return plan.ok;
}

Eigen::Vector3d TaskController::endEffectorPoint() const {
  return grip_point_in_world(_arm->config(), _geometry, currentPose(), 0.0, 0.0);
}

Eigen::Vector3d TaskController::assisted(const Eigen::Vector3d &point, double *hand_angle, double seconds) {
  if (!_assist || _heldId != -1) {
    forgetIntent();
    return point;
  }
  // Where the person is asking the hand to be: the asked-for end-effector
  // point plus the hand's own offset to where it grips (from the current
  // pose - it turns slowly compared with the hand's travel).
  const Eigen::Vector3d tip = endEffectorPoint();
  const Eigen::Vector3d hand = point + (handGripPoint(kTypicalGripRadius) - tip);
  std::vector<IntentEstimator::Candidate> candidates;
  for (const SceneObject &object : _objects) {
    candidates.push_back({object.id, object.base + Eigen::Vector3d(0.0, shapeOf(object.type, object.mass).grip_height, 0.0)});
  }
  _intent.update(hand, seconds, candidates);
  _guess = _intent.likely();

  _assistStrength = 0.0;
  Eigen::Vector3d result = point;
  if (const SceneObject *object = findObject(_guess.id)) {
    const Shape shape = shapeOf(object->type, object->mass);
    const Eigen::Vector3d center = object->base + Eigen::Vector3d(0.0, shape.grip_height, 0.0);
    const Eigen::Vector3d offset = handGripPoint(shape.grip_radius) - tip;
    _assistStrength = assist_strength(_guess.confidence, (point + offset - center).norm());
    // Toward the end-effector point that puts the object in the hand, and
    // an upright object's hand toward level.
    result += _assistStrength * ((center - offset) - point);
    if (hand_angle != nullptr && shape.upright) {
      *hand_angle *= 1.0 - _assistStrength;
    }
  }
  emit intentChanged();
  return result;
}

void TaskController::forgetIntent() {
  _intent.reset();
  const bool had = _guess.id != -1 || _assistStrength != 0.0;
  _guess = {};
  _assistStrength = 0.0;
  if (had) {
    emit intentChanged();
  }
}

void TaskController::setAssist(bool on) {
  if (on == _assist) {
    return;
  }
  _assist = on;
  if (!on) {
    forgetIntent();
    _finishingId = -1;
  }
  emit stateChanged();
}

bool TaskController::finishGrab() {
  const SceneObject *object = findObject(_guess.id);
  if (!_assist || object == nullptr || _guess.confidence < kFinishConfidence) {
    return false;
  }
  const Shape shape = shapeOf(object->type, object->mass);
  const Eigen::Vector3d center = object->base + Eigen::Vector3d(0.0, shape.grip_height, 0.0);
  if ((handGripPoint(shape.grip_radius) - center).norm() > kFinishReach) {
    return false;
  }
  return moveInToGrab(*object);
}

bool TaskController::moveInToGrab(const SceneObject &object) {
  // The grasp grabNearHand will hold it by: the hand from the side, the
  // gripper level.
  const Shape shape = shapeOf(object.type, object.mass);
  const GraspOption option =
      graspOption(object, object.base, industrial() ? QStringLiteral("side") : QStringLiteral("hand"), shape.grip_height);
  const PickPlan plan = plan_grasp(_arm->config(), _geometry, option.target, _pose.hip_height);
  if (!plan.ok) {
    return false;
  }
  _goal = plan.pose;
  _goal.hand_roll = 0.0;
  _finishingId = object.id;
  _following = true;
  startTicker();
  setStatus(QStringLiteral("Taking the %1").arg(object.type));
  return true;
}

const TaskController::SceneObject *TaskController::nearestToHand(double reach) const {
  // Each object against where it would sit in the hand.
  const SceneObject *nearest = nullptr;
  double nearest_distance = reach;
  for (const SceneObject &object : _objects) {
    const Shape shape = shapeOf(object.type, object.mass);
    const Eigen::Vector3d grip_center = object.base + Eigen::Vector3d(0.0, shape.grip_height, 0.0);
    const double distance = (handGripPoint(shape.grip_radius) - grip_center).norm();
    if (!isHeld(object.id) && distance < nearest_distance) {
      nearest = &object;
      nearest_distance = distance;
    }
  }
  return nearest;
}

bool TaskController::grabOrRelease() {
  if (busy() || _finishingId != -1) {
    return false;
  }
  if (_heldId != -1) {
    setGrip(0.0);
    return _heldId == -1;
  }
  const SceneObject *object = nearestToHand(kFinishReach);
  if (object == nullptr) {
    setStatus(QStringLiteral("Nothing close enough to grab - bring the hand within 20 cm of an object"));
    return false;
  }
  cancelReach();
  _gripClosed = true; // so opening (setGrip, or this again) lets go
  if (grabNearHand(false)) {
    return true;
  }
  if (!moveInToGrab(*object)) {
    _gripClosed = false;
    setStatus(QStringLiteral("Can't get the hand around the %1 from here").arg(object->type));
    return false;
  }
  return true;
}

void TaskController::showWhatsInReach(const Eigen::Vector3d &point) {
  if (_heldId != -1 || busy()) {
    forgetIntent();
    return;
  }
  // The object nearest where the hand is going (its grip point for that
  // object, from the end-effector point asked for), if any is in reach -
  // brighter the closer.
  const Eigen::Vector3d tip = endEffectorPoint();
  Intent nearest;
  double nearest_distance = kFinishReach;
  for (const SceneObject &object : _objects) {
    const Shape shape = shapeOf(object.type, object.mass);
    const Eigen::Vector3d center = object.base + Eigen::Vector3d(0.0, shape.grip_height, 0.0);
    const double distance = (point + (handGripPoint(shape.grip_radius) - tip) - center).norm();
    if (distance < nearest_distance) {
      nearest = {object.id, 1.0 - distance / kFinishReach};
      nearest_distance = distance;
    }
  }
  _guess = nearest;
  _assistStrength = 0.0;
  emit intentChanged();
}

bool TaskController::reachToward(const Eigen::Vector3d &point, const double *hand_angle) {
  if (busy()) {
    return false;
  }
  _finishingId = -1; // a new goal (a mouse drag...) takes over from finishing a grab
  // Plan from the previous goal, not from where the body happens to be
  // mid-motion, so successive goals of one drag stay consistent.
  const BodyPose from = _following ? _goal : currentPose();
  const PickPlan plan = plan_reach(_arm->config(), _geometry, point, from, hand_angle);
  if (plan.pose.arm.size() == _arm->config().joints.size()) {
    _goal = plan.pose;
    if (!_following) {
      _following = true;
      emit stateChanged();
    }
    startTicker();
  }
  return plan.ok;
}

void TaskController::setHandRoll(double radians) {
  if (busy() || industrial() || !std::isfinite(radians)) {
    return;
  }
  if (!_following) {
    _goal = currentPose();
    _following = true;
    emit stateChanged();
  }
  if (_finishingId != -1) {
    return; // the grasp's own (neutral) roll
  }
  // Pulled toward a grab, the hand turns back toward neutral as much.
  _goal.hand_roll = std::clamp(radians * (1.0 - _assistStrength), -kMaxHandRoll, kMaxHandRoll);
  startTicker();
}

void TaskController::setGrip(double amount) {
  if (busy()) {
    return;
  }
  _grip = std::clamp(amount, 0.0, 1.0);

  if (!_gripClosed && _grip >= kGrabAt) {
    _gripClosed = true;
    if (_heldId == -1 && !grabNearHand(false) && !finishGrab()) {
      setStatus(QStringLiteral("Nothing in the hand to grab - bring it closer to an object"));
    }
  } else if (_gripClosed && _grip <= kReleaseAt) {
    if (_finishingId != -1) {
      _finishingId = -1; // opened again before the hand got there: never mind
      cancelReach();
    }
    // Not over a place it can go: keep holding, and try again on the next
    // update - it lets go as soon as the hand moves somewhere it can.
    _gripClosed = _heldId != -1 && !releaseUnderHand();
  }

  // Holding something, the fingers stay wrapped around it (closing harder
  // can't go through it; letting go is a separate step). Empty, they
  // follow the grip.
  const SceneObject *held = findObject(_heldId);
  if (industrial()) {
    _gripperWidth = held != nullptr ? 2.0 * shapeOf(held->type, held->mass).grip_radius
                                    : kGripperOpen - _grip * (kGripperOpen - kGripperShut);
  } else if (_hand != nullptr) {
    _hand->setGrasp(held != nullptr ? gripClosure(shapeOf(held->type, held->mass).grip_radius) : _grip);
  }
  emit stateChanged();
}

Eigen::Vector3d TaskController::handGripPoint(double grip_radius) const {
  const ArmConfig &config = _arm->config();
  if (industrial()) {
    return grip_point_in_world(config, _geometry, currentPose(), 0.0, kGripperReach);
  }
  const double hand_link = config.joints.empty() ? 0.0 : config.joints.back().dh.a;
  return grip_point_in_world(config, _geometry, currentPose(), gripPalmar(grip_radius),
                             gripForward(grip_radius) - hand_link);
}

bool TaskController::grabNearHand(bool complain) {
  const SceneObject *found = nearestToHand(kGrabReach);
  SceneObject *nearest = found != nullptr ? findObject(found->id) : nullptr;
  if (nearest == nullptr) {
    if (complain) {
      setStatus(QStringLiteral("Nothing in the hand to grab - bring it closer to an object"));
    }
    return false;
  }

  const Shape shape = shapeOf(nearest->type, nearest->mass);
  nearest->grasp_mode = industrial() ? QStringLiteral("side") : QStringLiteral("hand");
  nearest->grasp_height = shape.grip_height;
  _heldId = nearest->id;
  _arm->setPayloadKg(nearest->mass);
  _arm->setGripperOpen(false);
  if (industrial()) {
    _gripperWidth = 2.0 * shape.grip_radius;
  } else if (_hand != nullptr) {
    _hand->setGrasp(gripClosure(shape.grip_radius));
  }
  forgetIntent();
  setStatus(QStringLiteral("Holding the %1 (%2)").arg(nearest->type, massText(nearest->mass)));
  emit objectsChanged();
  emit stateChanged();
  return true;
}

bool TaskController::releaseUnderHand() {
  SceneObject *object = findObject(_heldId);
  if (object == nullptr) {
    return false;
  }
  // It goes straight down from where it is in the hand.
  const Shape shape = shapeOf(object->type, object->mass);
  const Eigen::Vector3d center = handGripPoint(shape.grip_radius);
  const double support = supportHeightAt(center.x(), center.z(), shape.footprint);
  if (support < 0.0) {
    setStatus(QStringLiteral("Can't let go here - over a table edge or the robot's base"));
    return false;
  }
  if (!spotIsFree(center.x(), center.z(), shape.footprint, object->id)) {
    setStatus(QStringLiteral("Can't let go here - another object is right below"));
    return false;
  }

  object->base = Eigen::Vector3d(center.x(), support, center.z());
  object->grasp_mode.clear();
  _heldId = -1;
  _arm->setPayloadKg(0.0);
  _arm->setGripperOpen(true);
  setStatus(QStringLiteral("Put the %1 down").arg(object->type));
  emit objectsChanged();
  emit stateChanged();
  return true;
}

void TaskController::cancelReach() {
  _finishingId = -1;
  _goalSpeed.clear(); // stopped where it is
  if (!_following) {
    return;
  }
  _following = false;
  stopTickerIfIdle();
  emit stateChanged();
}

void TaskController::followGoal(double seconds) {
  const BodyPose now = currentPose();
  if (now.arm.size() != _goal.arm.size()) {
    cancelReach(); // the arm changed under the goal (a joint added...)
    return;
  }

  // The gap per degree of freedom, and each one's speed and acceleration
  // limits.
  std::vector<double> gap = {wrap(_goal.yaw - now.yaw), _goal.hip_height - now.hip_height,
                             _goal.hip_bend - now.hip_bend, _goal.hand_roll - now.hand_roll};
  std::vector<double> top = {kMaxYawSpeed, kMaxLiftSpeed, kMaxBendSpeed, kMaxRollSpeed};
  std::vector<double> top_accel = {kMaxYawAccel, kMaxLiftAccel, kMaxBendAccel, kMaxRollAccel};
  for (size_t i = 0; i < now.arm.size(); i++) {
    gap.push_back(_goal.arm[i] - now.arm[i]);
    top.push_back(kMaxJointSpeed);
    top_accel.push_back(kMaxJointAccel);
  }
  if (_goalSpeed.size() != gap.size()) {
    _goalSpeed.assign(gap.size(), 0.0);
  }

  double remaining = 0.0;
  for (const double g : gap) {
    remaining = std::max(remaining, std::abs(g));
  }
  if (remaining < kArrived) {
    _goalSpeed.assign(gap.size(), 0.0);
    applyPose(_goal);
    if (_finishingId != -1) {
      _finishingId = -1;
      grabNearHand();
    }
    cancelReach();
    finishLeftGrabIfArrived();
    return;
  }

  const FollowStep step = followStep(gap, _goalSpeed, top, top_accel, seconds);
  _goalSpeed = step.speed;
  BodyPose next = now;
  next.yaw = now.yaw + step.move[0];
  next.hip_height = now.hip_height + step.move[1];
  next.hip_bend = now.hip_bend + step.move[2];
  next.hand_roll = now.hand_roll + step.move[3];
  for (size_t i = 0; i < now.arm.size(); i++) {
    next.arm[i] = now.arm[i] + step.move[4 + i];
  }
  applyPose(next);
}

void TaskController::stopTickerIfIdle() {
  if (_steps.empty() && !_following && !_leftFollowing) {
    _ticker.stop();
  }
}

void TaskController::startTicker() {
  if (!_ticker.isActive()) {
    _tickClock.start();
    _ticker.start();
  }
}

bool TaskController::useIndustrialArm() {
  if (busy() || _heldId != -1) {
    setStatus(QStringLiteral("Put the object down before changing the arm"));
    return false;
  }
  cancelReach();
  _arm->convertToIndustrial();
  _gripperWidth = kGripperOpen;
  setStatus(QStringLiteral("Warehouse arm with a gripper - add joints in the Arm card"));
  return true;
}

bool TaskController::useHumanoidArm() {
  if (busy() || _heldId != -1) {
    setStatus(QStringLiteral("Put the object down before changing the arm"));
    return false;
  }
  cancelReach();
  if (!_arm->reloadInitialConfig()) {
    return false;
  }
  applyPose(homePose(_pose.yaw));
  _hand->playGesture(QStringLiteral("restPose"));
  setStatus(QStringLiteral("Humanoid arm"));
  return true;
}

BodyPose TaskController::sideOffPose(BodyPose pose, const Eigen::Vector3d &grip_point, double palmar_offset,
                                     double clearance) const {
  double forward = 0.0;
  const std::optional<double> yaw = yaw_to_place_point_in_plane(
      grip_point, _geometry.shoulder_lateral - palmar_offset - clearance, forward);
  if (yaw.has_value()) {
    pose.yaw = *yaw;
  }
  return pose;
}

double TaskController::stepDuration(const BodyPose &from, const BodyPose &to) const {
  double seconds = std::abs(wrap(to.yaw - from.yaw)) * 0.9;
  seconds = std::max(seconds, std::abs(to.hip_height - from.hip_height) * 3.0);
  seconds = std::max(seconds, std::abs(to.hip_bend - from.hip_bend) * 1.2);
  seconds = std::max(seconds, std::abs(to.hand_roll - from.hand_roll) * 0.5);
  for (size_t i = 0; i < std::min(from.arm.size(), to.arm.size()); i++) {
    seconds = std::max(seconds, std::abs(to.arm[i] - from.arm[i]) * 0.8);
  }
  return std::clamp(seconds + 0.5, 0.7, 2.6);
}

void TaskController::queue(const BodyPose &target, double duration, std::function<void()> on_arrive) {
  _steps.push_back({target, duration, std::move(on_arrive)});
  startTicker();
  emit stateChanged();
}

bool TaskController::pickUp(int id) {
  if (busy()) {
    setStatus(QStringLiteral("Busy - wait for the current move to finish"));
    return false;
  }
  if (_heldId != -1) {
    setStatus(QStringLiteral("Already holding something - put it down first"));
    return false;
  }
  if (isHeld(id)) {
    setStatus(QStringLiteral("The left hand is holding that"));
    return false;
  }
  const SceneObject *object = findObject(id);
  if (object == nullptr) {
    return false;
  }
  cancelReach();

  // First grasp option that works.
  GraspOption option;
  PickPlan grasp;
  QString reason;
  for (const GraspOption &candidate : graspOptions(*object, object->base)) {
    grasp = plan_grasp(_arm->config(), _geometry, candidate.target, _pose.hip_height);
    if (grasp.ok) {
      option = candidate;
      break;
    }
    if (reason.isEmpty()) {
      reason = QString::fromStdString(grasp.reason);
    }
  }
  if (!grasp.ok) {
    setStatus(QStringLiteral("Can't pick up the %1: %2").arg(object->type, reason));
    return false;
  }
  option.target.hand_angles = {grasp.hand_angle}; // keep the hand the way it grasped from here on

  GraspTarget raised = option.target;
  raised.grip_point.y() += kLiftHeight;
  PickPlan lifted = plan_grasp(_arm->config(), _geometry, raised, grasp.pose.hip_height);
  if (!lifted.ok) {
    lifted = grasp; // can't raise it any further - just hold it where it is
  }

  const Shape shape = shapeOf(object->type, object->mass);
  const BodyPose start = currentPose();
  const BodyPose approach = approachPose(option, grasp, kApproachClearance);
  const BodyPose carry = carryPose(option, lifted);
  const double closure = gripClosure(shape.grip_radius);
  const double closed_width = 2.0 * shape.grip_radius;
  const double mass = object->mass;
  const QString type = object->type;
  const QString mode = option.mode;
  const double height = option.height;

  _selectedId = id;
  _gripperWidth = kGripperOpen;
  _arm->setGripperOpen(true);
  _hand->playGesture(QStringLiteral("openPalm"));
  setStatus(QStringLiteral("Reaching for the %1...").arg(type));

  queue(approach, stepDuration(start, approach));
  queue(grasp.pose, 0.9, [this, closure, closed_width]() {
    _hand->playGrasp(closure);
    _gripperWidth = closed_width;
    emit stateChanged();
  });
  queue(grasp.pose, 0.9, [this, id, mass, mode, height]() {
    if (SceneObject *held = findObject(id)) {
      held->grasp_mode = mode;
      held->grasp_height = height;
    }
    _heldId = id;
    _arm->setPayloadKg(mass);
    _arm->setGripperOpen(false);
    emit objectsChanged();
  });
  queue(lifted.pose, stepDuration(grasp.pose, lifted.pose));
  queue(carry, stepDuration(lifted.pose, carry), [this, type, mass]() {
    setStatus(QStringLiteral("Holding the %1 (%2)").arg(type, massText(mass)));
  });
  return true;
}

bool TaskController::putDown(double x, double z) {
  if (busy()) {
    setStatus(QStringLiteral("Busy - wait for the current move to finish"));
    return false;
  }
  SceneObject *object = findObject(_heldId);
  if (object == nullptr) {
    setStatus(QStringLiteral("Not holding anything"));
    return false;
  }

  const Shape shape = shapeOf(object->type, object->mass);
  const double support = supportHeightAt(x, z, shape.footprint);
  if (support < 0.0) {
    setStatus(QStringLiteral("Can't put it there (table edge or the robot's base)"));
    return false;
  }
  if (!spotIsFree(x, z, shape.footprint, object->id)) {
    setStatus(QStringLiteral("Another object is already there"));
    return false;
  }

  // Put it down the same way it's held.
  const Eigen::Vector3d base(x, support, z);
  GraspOption option = graspOption(*object, base, object->grasp_mode.isEmpty() ? QStringLiteral("hand")
                                                                               : object->grasp_mode,
                                   object->grasp_mode.isEmpty() ? shape.grip_height : object->grasp_height);
  const PickPlan place = plan_grasp(_arm->config(), _geometry, option.target, _pose.hip_height);
  if (!place.ok) {
    setStatus(QStringLiteral("Can't put the %1 there: %2").arg(object->type, QString::fromStdString(place.reason)));
    return false;
  }
  option.target.hand_angles = {place.hand_angle};
  GraspTarget raised = option.target;
  raised.grip_point.y() += kLiftHeight;
  PickPlan above = plan_grasp(_arm->config(), _geometry, raised, place.pose.hip_height);
  if (!above.ok) {
    above = place;
  }

  cancelReach();
  const BodyPose start = currentPose();
  const BodyPose retreat = approachPose(option, place, kApproachClearance);
  const int id = object->id;
  const QString type = object->type;
  setStatus(QStringLiteral("Putting the %1 down...").arg(type));

  queue(above.pose, stepDuration(start, above.pose));
  queue(place.pose, 0.9, [this, id, base]() {
    // Let go: the object now stands where the hand put it.
    if (SceneObject *placed = findObject(id)) {
      placed->base = base;
      placed->grasp_mode.clear();
    }
    _heldId = -1;
    _gripperWidth = kGripperOpen;
    _arm->setPayloadKg(0.0);
    _arm->setGripperOpen(true);
    _hand->playGesture(QStringLiteral("openPalm"));
    emit objectsChanged();
    emit stateChanged();
  });
  queue(place.pose, 0.6);
  queue(retreat, 0.9, [this, type]() {
    // Home is computed now, from wherever the arm actually ended up.
    const BodyPose home = homePose(0.0);
    queue(home, stepDuration(currentPose(), home), [this, type]() {
      _hand->playGesture(QStringLiteral("restPose"));
      setStatus(QStringLiteral("Put the %1 down").arg(type));
    });
  });
  return true;
}

void TaskController::stop() {
  cancelReach();
  if (_steps.empty()) {
    return;
  }
  _steps.clear();
  _stepStarted = false;
  stopTickerIfIdle();
  setStatus(QStringLiteral("Stopped"));
  emit stateChanged();
}

void TaskController::advance(double seconds) {
  if (_leftFollowing && seconds > 0.0) {
    followLeft(seconds);
  }
  if (_steps.empty()) {
    if (_following && seconds > 0.0) {
      followGoal(seconds);
    }
    return;
  }
  while (seconds > 0.0 && !_steps.empty()) {
    if (!_stepStarted) {
      _stepStart = currentPose();
      _stepElapsed = 0.0;
      _stepStarted = true;
    }
    Step &step = _steps.front();
    const double used = std::min(seconds, step.duration - _stepElapsed);
    _stepElapsed += std::max(0.0, used);
    seconds -= std::max(used, 1e-12);

    // Smoothstep easing: starts and stops gently, like a real joint.
    const double u = step.duration > 0.0 ? std::clamp(_stepElapsed / step.duration, 0.0, 1.0) : 1.0;
    const double s = u * u * (3.0 - 2.0 * u);

    BodyPose pose;
    pose.yaw = _stepStart.yaw + s * wrap(step.target.yaw - _stepStart.yaw);
    pose.hip_height = _stepStart.hip_height + s * (step.target.hip_height - _stepStart.hip_height);
    pose.hip_bend = _stepStart.hip_bend + s * (step.target.hip_bend - _stepStart.hip_bend);
    pose.hand_roll = _stepStart.hand_roll + s * (step.target.hand_roll - _stepStart.hand_roll);
    pose.arm = _stepStart.arm;
    for (size_t i = 0; i < std::min(pose.arm.size(), step.target.arm.size()); i++) {
      pose.arm[i] = _stepStart.arm[i] + s * (step.target.arm[i] - _stepStart.arm[i]);
    }
    applyPose(pose);

    if (u >= 1.0) {
      std::function<void()> on_arrive = std::move(step.on_arrive);
      _steps.pop_front();
      _stepStarted = false;
      if (on_arrive) {
        on_arrive();
      }
      if (_steps.empty()) {
        stopTickerIfIdle();
        emit stateChanged();
      }
    }
  }
}

// ---- clicks + status -----------------------------------------------------

void TaskController::setClickMode(const QString &mode) {
  if (mode == _clickMode) {
    return;
  }
  _clickMode = mode;
  emit stateChanged();
}

void TaskController::setPlaceType(const QString &type) {
  if (type == _placeType || !objectTypes().contains(type)) {
    return;
  }
  _placeType = type;
  _placeMassKg = defaultMass(type);
  emit stateChanged();
}

void TaskController::setPlaceMassKg(double kg) {
  if (kg <= 0.0 || kg == _placeMassKg) {
    return;
  }
  _placeMassKg = kg;
  emit stateChanged();
}

void TaskController::handleClick(double x, double z) {
  if (_clickMode == QStringLiteral("place")) {
    if (addObject(_placeType, _placeMassKg, x, z) != -1) {
      setClickMode(QString());
    }
  } else if (_clickMode == QStringLiteral("move")) {
    if (moveObject(_selectedId, x, z)) {
      setClickMode(QString());
    }
  } else if (_clickMode == QStringLiteral("putDown")) {
    if (putDown(x, z)) {
      setClickMode(QString());
    }
  } else {
    selectAt(x, z);
  }
}

void TaskController::setStatus(const QString &status) {
  _status = status;
  emit stateChanged();
}

// ---- the left arm ----------------------------------------------------------
//
// Planned as the right arm of the mirrored robot (see mirrored() in
// core/pick_planner): mirror the point and the pose, plan, mirror back.

void TaskController::setLeftArm(ArmController *arm, HandController *hand) {
  _leftArm = arm;
  _leftHand = hand;
  if (_leftArm != nullptr) {
    _leftArm->setBaseTiltDegrees(to_degrees(_pose.hip_bend));
    if (_leftArm->jointCount() == 3) {
      // Hanging relaxed at the side until it's used.
      _leftArm->setJointAnglesRadians({to_radians(-85.0), to_radians(15.0), 0.0});
    }
    _leftGoalArm = _leftArm->jointAnglesRadians();
  }
  emit leftArmChanged();
}

QVariantMap TaskController::leftHeldObject() const {
  const SceneObject *object = findObject(_leftHeldId);
  return object == nullptr ? QVariantMap() : toVariant(*object);
}

double TaskController::leftHandRollDegrees() const { return to_degrees(_leftRoll); }

BodyPose TaskController::leftPose() const {
  BodyPose pose = _pose;
  pose.arm = _leftArm->jointAnglesRadians();
  pose.hand_roll = _leftRoll;
  return pose;
}

Eigen::Vector3d TaskController::leftGripPoint(double grip_radius) const {
  const ArmConfig &config = _leftArm->config();
  const double hand_link = config.joints.empty() ? 0.0 : config.joints.back().dh.a;
  return mirrored(grip_point_in_world(config, _geometry, mirrored(leftPose()), gripPalmar(grip_radius),
                                      gripForward(grip_radius) - hand_link));
}

bool TaskController::reachLeft(double x, double y, double z, double handAngleRadians) {
  noteMove(_leftMover, Eigen::Vector3d(x, y, z));
  return reachLeft(x, y, z, handAngleRadians, leftLeads());
}

void TaskController::noteMove(Mover &mover, const Eigen::Vector3d &target) {
  if (!mover.anchored) {
    mover.anchor = target; // where it starts isn't a move
    mover.anchored = true;
  } else if ((target - mover.anchor).norm() >= kMoved) {
    mover.anchor = target;
    mover.moved.start();
  }
}

bool TaskController::leftLeads() const {
  return _leftArm != nullptr && _leftMover.moved.isValid() &&
         (!_rightMover.moved.isValid() || _leftMover.moved.nsecsElapsed() < _rightMover.moved.nsecsElapsed());
}

bool TaskController::reachLeft(double x, double y, double z, double handAngleRadians, bool leads) {
  if (_leftArm == nullptr || busy() || _leftFinishingId != -1) {
    return false;
  }
  leads = leads && _finishingId == -1; // the right hand is moving in for a grab: the body is its
  double hand_angle = handAngleRadians;
  const double *angle = std::isfinite(hand_angle) ? &hand_angle : nullptr;
  // From the goals both are heading for, so successive targets stay
  // consistent (as reachToward does).
  BodyPose from = _following ? _goal : currentPose();
  from.arm = _leftFollowing ? _leftGoalArm : _leftArm->jointAnglesRadians();
  from.hand_roll = _leftGoalRoll;
  const Eigen::Vector3d point = mirrored(Eigen::Vector3d(x, y, z));

  PickPlan plan;
  if (leads) {
    plan = plan_reach(_leftArm->config(), _geometry, point, mirrored(from), angle);
    if (plan.pose.arm.size() != from.arm.size()) {
      return false;
    }
    // The body goes where the left arm needs it; the right arm rides along.
    const BodyPose body = mirrored(plan.pose);
    BodyPose goal = _following ? _goal : currentPose();
    goal.yaw = body.yaw;
    goal.hip_height = body.hip_height;
    goal.hip_bend = body.hip_bend;
    _goal = goal;
    if (!_following) {
      _following = true;
      emit stateChanged();
    }
  } else {
    plan = plan_arm_reach(_leftArm->config(), _geometry, point, mirrored(from), angle);
  }
  _leftGoalArm = plan.pose.arm;
  if (!_leftFollowing) {
    _leftFollowing = true;
    emit stateChanged();
  }
  startTicker();
  return plan.ok;
}

void TaskController::setLeftHandRoll(double radians) {
  if (_leftArm == nullptr || busy() || !std::isfinite(radians) || _leftFinishingId != -1) {
    return;
  }
  _leftGoalRoll = std::clamp(radians, -kMaxHandRoll, kMaxHandRoll);
  if (!_leftFollowing) {
    _leftGoalArm = _leftArm->jointAnglesRadians();
    _leftFollowing = true;
  }
  startTicker();
}

void TaskController::cancelLeftReach() {
  _leftFinishingId = -1;
  _leftGoalSpeed.clear();
  if (!_leftFollowing) {
    return;
  }
  _leftFollowing = false;
  stopTickerIfIdle();
  emit stateChanged();
}

void TaskController::followLeft(double seconds) {
  const std::vector<double> now = _leftArm->jointAnglesRadians();
  if (now.size() != _leftGoalArm.size()) {
    cancelLeftReach();
    return;
  }
  // Like followGoal: exponential, no joint faster or harder than it can
  // go, all finishing together.
  std::vector<double> gap = {_leftGoalRoll - _leftRoll};
  std::vector<double> top = {kMaxRollSpeed};
  std::vector<double> top_accel = {kMaxRollAccel};
  for (size_t i = 0; i < now.size(); i++) {
    gap.push_back(_leftGoalArm[i] - now[i]);
    top.push_back(kMaxJointSpeed);
    top_accel.push_back(kMaxJointAccel);
  }
  if (_leftGoalSpeed.size() != gap.size()) {
    _leftGoalSpeed.assign(gap.size(), 0.0);
  }
  double remaining = 0.0;
  for (const double g : gap) {
    remaining = std::max(remaining, std::abs(g));
  }
  if (remaining < kArrived) {
    _leftGoalSpeed.assign(gap.size(), 0.0);
    _leftRoll = _leftGoalRoll;
    _leftArm->setJointAnglesRadians(_leftGoalArm);
    _leftFollowing = false;
    emit bodyChanged();
    emit stateChanged();
    stopTickerIfIdle();
    finishLeftGrabIfArrived();
    return;
  }
  const FollowStep step = followStep(gap, _leftGoalSpeed, top, top_accel, seconds);
  _leftGoalSpeed = step.speed;
  _leftRoll += step.move[0];
  std::vector<double> next = now;
  for (size_t i = 0; i < now.size(); i++) {
    next[i] += step.move[1 + i];
  }
  _leftArm->setJointAnglesRadians(next);
  emit bodyChanged();
}

const TaskController::SceneObject *TaskController::nearestToLeftHand(double reach) const {
  const SceneObject *nearest = nullptr;
  double nearest_distance = reach;
  for (const SceneObject &object : _objects) {
    const Shape shape = shapeOf(object.type, object.mass);
    const Eigen::Vector3d grip_center = object.base + Eigen::Vector3d(0.0, shape.grip_height, 0.0);
    const double distance = (leftGripPoint(shape.grip_radius) - grip_center).norm();
    if (!isHeld(object.id) && distance < nearest_distance) {
      nearest = &object;
      nearest_distance = distance;
    }
  }
  return nearest;
}

void TaskController::setLeftGrip(double amount) {
  if (_leftArm == nullptr || busy()) {
    return;
  }
  _leftGrip = std::clamp(amount, 0.0, 1.0);
  if (!_leftGripClosed && _leftGrip >= kGrabAt) {
    _leftGripClosed = true;
    if (_leftHeldId == -1 && !grabNearLeftHand()) {
      const SceneObject *near = nearestToLeftHand(kFinishReach);
      if (near == nullptr || !leftMoveInToGrab(*near)) {
        setStatus(QStringLiteral("Nothing in the left hand to grab - bring it closer to an object"));
      }
    }
  } else if (_leftGripClosed && _leftGrip <= kReleaseAt) {
    if (_leftFinishingId != -1) {
      _leftFinishingId = -1; // opened before it got there
      cancelLeftReach();
    }
    _leftGripClosed = _leftHeldId != -1 && !releaseUnderLeftHand();
  }
  const SceneObject *held = findObject(_leftHeldId);
  if (_leftHand != nullptr) {
    _leftHand->setGrasp(held != nullptr ? gripClosure(shapeOf(held->type, held->mass).grip_radius) : _leftGrip);
  }
  emit stateChanged();
}

bool TaskController::grabNearLeftHand() {
  const SceneObject *found = nearestToLeftHand(kGrabReach);
  SceneObject *object = found != nullptr ? findObject(found->id) : nullptr;
  if (object == nullptr) {
    return false;
  }
  const Shape shape = shapeOf(object->type, object->mass);
  object->grasp_mode = QStringLiteral("hand");
  object->grasp_height = shape.grip_height;
  _leftHeldId = object->id;
  _leftArm->setPayloadKg(object->mass);
  _leftArm->setGripperOpen(false);
  if (_leftHand != nullptr) {
    _leftHand->setGrasp(gripClosure(shape.grip_radius));
  }
  setStatus(QStringLiteral("Left hand holding the %1 (%2)").arg(object->type, massText(object->mass)));
  emit objectsChanged();
  emit stateChanged();
  return true;
}

bool TaskController::releaseUnderLeftHand() {
  SceneObject *object = findObject(_leftHeldId);
  if (object == nullptr) {
    return false;
  }
  const Shape shape = shapeOf(object->type, object->mass);
  const Eigen::Vector3d center = leftGripPoint(shape.grip_radius);
  const double support = supportHeightAt(center.x(), center.z(), shape.footprint);
  if (support < 0.0) {
    setStatus(QStringLiteral("Can't let go here - over a table edge or the robot's base"));
    return false;
  }
  if (!spotIsFree(center.x(), center.z(), shape.footprint, object->id)) {
    setStatus(QStringLiteral("Can't let go here - another object is right below"));
    return false;
  }
  object->base = Eigen::Vector3d(center.x(), support, center.z());
  object->grasp_mode.clear();
  _leftHeldId = -1;
  _leftArm->setPayloadKg(0.0);
  _leftArm->setGripperOpen(true);
  setStatus(QStringLiteral("Left hand put the %1 down").arg(object->type));
  emit objectsChanged();
  emit stateChanged();
  return true;
}

bool TaskController::leftMoveInToGrab(const SceneObject &object) {
  // The grasp as the mirrored robot's right hand would make it.
  const Shape shape = shapeOf(object.type, object.mass);
  GraspOption option = graspOption(object, object.base, QStringLiteral("hand"), shape.grip_height);
  option.target.grip_point = mirrored(option.target.grip_point);
  const PickPlan plan = plan_grasp(_leftArm->config(), _geometry, option.target, _pose.hip_height);
  if (!plan.ok) {
    return false;
  }
  const BodyPose pose = mirrored(plan.pose);
  BodyPose goal = _following ? _goal : currentPose();
  goal.yaw = pose.yaw;
  goal.hip_height = pose.hip_height;
  goal.hip_bend = pose.hip_bend;
  _finishingId = -1;
  _goal = goal;
  _following = true;
  _leftGoalArm = pose.arm;
  _leftGoalRoll = 0.0;
  _leftFollowing = true;
  _leftFinishingId = object.id;
  startTicker();
  emit stateChanged();
  setStatus(QStringLiteral("Left hand taking the %1").arg(object.type));
  return true;
}

void TaskController::finishLeftGrabIfArrived() {
  if (_leftFinishingId == -1 || _leftFollowing || _following) {
    return;
  }
  _leftFinishingId = -1;
  grabNearLeftHand();
}

} // namespace robotarm
