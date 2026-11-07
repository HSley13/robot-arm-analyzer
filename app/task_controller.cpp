#include "task_controller.hpp"

#include <algorithm>
#include <cmath>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

namespace {

constexpr double kApproachClearance = 0.10; // m, hand starts/ends this far from the object, palm toward it
constexpr double kLiftHeight = 0.12;        // m, how far a picked object is raised (and lowered onto a spot)
constexpr double kTickSeconds = 1.0 / 60.0;
constexpr double kStandingHipHeight = 0.80; // m, the lift's normal standing height
constexpr double kGripperReach = 0.075;     // m, gripper finger center past the arm's flange
constexpr double kGripperOpen = 0.11;       // m, gripper fingers wide open

double wrap(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
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
  return {QStringLiteral("bottle"), QStringLiteral("can"), QStringLiteral("box"), QStringLiteral("ball"),
          QStringLiteral("dumbbell")};
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
  if (type == QStringLiteral("dumbbell")) {
    return 8.0;
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
  map["held"] = object.id == _heldId;
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
  const bool fully_on_table = x >= kTableX0 + footprint && x <= kTableX1 - footprint && z >= kTableZ0 + footprint &&
                              z <= kTableZ1 - footprint;
  if (fully_on_table) {
    return kTableHeight;
  }
  const double dx = std::max({kTableX0 - x, 0.0, x - kTableX1});
  const double dz = std::max({kTableZ0 - z, 0.0, z - kTableZ1});
  if (std::hypot(dx, dz) > footprint) {
    return 0.0; // clear of the table: the floor
  }
  return -1.0; // straddling the table's edge
}

bool TaskController::spotIsFree(double x, double z, double footprint, int ignore_id) const {
  for (const SceneObject &other : _objects) {
    if (other.id == ignore_id || other.id == _heldId) {
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
  if (object == nullptr || id == _heldId) {
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
  }
  emit objectsChanged();
}

void TaskController::removeObject(int id) {
  if (id == _heldId || busy()) {
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
    if (object.id == _heldId) {
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
double TaskController::hipBendDegrees() const { return to_degrees(_pose.hip_bend); }

void TaskController::setWaistYawDegrees(double degrees) {
  if (busy()) {
    return;
  }
  BodyPose pose = currentPose();
  pose.yaw = to_radians(std::clamp(degrees, _geometry.min_yaw_degrees, _geometry.max_yaw_degrees));
  applyPose(pose);
}

void TaskController::setHipHeight(double meters) {
  if (busy()) {
    return;
  }
  BodyPose pose = currentPose();
  pose.hip_height = std::clamp(meters, _geometry.min_hip_height, _geometry.max_hip_height);
  applyPose(pose);
}

void TaskController::setHipBendDegrees(double degrees) {
  if (busy()) {
    return;
  }
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
  return map;
}

QVariantMap TaskController::table() const {
  QVariantMap map;
  map["x0"] = kTableX0;
  map["x1"] = kTableX1;
  map["z0"] = kTableZ0;
  map["z1"] = kTableZ1;
  map["height"] = kTableHeight;
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
  _arm->setBaseTiltDegrees(to_degrees(pose.hip_bend));
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
  // whose top plate is in the way.
  if (object.type != QStringLiteral("dumbbell")) {
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
  if (busy()) {
    return false;
  }
  const PickPlan plan = plan_reach(_arm->config(), _geometry, Eigen::Vector3d(x, y, z), currentPose());
  if (plan.pose.arm.size() == _arm->config().joints.size()) {
    applyPose(plan.pose);
  }
  return plan.ok;
}

bool TaskController::useIndustrialArm() {
  if (busy() || _heldId != -1) {
    setStatus(QStringLiteral("Put the object down before changing the arm"));
    return false;
  }
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
  for (size_t i = 0; i < std::min(from.arm.size(), to.arm.size()); i++) {
    seconds = std::max(seconds, std::abs(to.arm[i] - from.arm[i]) * 0.8);
  }
  return std::clamp(seconds + 0.5, 0.7, 2.6);
}

void TaskController::queue(const BodyPose &target, double duration, std::function<void()> on_arrive) {
  _steps.push_back({target, duration, std::move(on_arrive)});
  if (!_ticker.isActive()) {
    _tickClock.start();
    _ticker.start();
  }
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
  const SceneObject *object = findObject(id);
  if (object == nullptr) {
    return false;
  }

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
  if (_steps.empty()) {
    return;
  }
  _steps.clear();
  _stepStarted = false;
  _ticker.stop();
  setStatus(QStringLiteral("Stopped"));
  emit stateChanged();
}

void TaskController::advance(double seconds) {
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
        _ticker.stop();
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

} // namespace robotarm
