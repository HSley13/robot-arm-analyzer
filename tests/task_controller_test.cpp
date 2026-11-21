#include <QVector3D>

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "robotarm/pick_planner.hpp"
#include "task_controller.hpp"

using robotarm::ArmController;
using robotarm::HandController;
using robotarm::TaskController;

namespace {

struct Rig {
  ArmController arm{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController hand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  TaskController task{&arm, &hand};

  // Runs the queued sequence to the end in fixed 1/60 s steps, like the
  // app's timer would.
  void runToEnd() {
    for (int i = 0; i < 60 * 30 && (task.busy() || task.reaching()); i++) {
      task.advance(1.0 / 60.0);
    }
  }
};

QVariantMap objectById(const TaskController &task, int id) {
  for (const QVariant &value : task.objects()) {
    const QVariantMap map = value.toMap();
    if (map["id"].toInt() == id) {
      return map;
    }
  }
  return {};
}

// Where the arm's tip is in the world, from the drawn frames: arm plane
// at the right shoulder, torso leaning at the hip, the waist turning it.
QVector3D tipInWorld(const ArmController &arm, const TaskController &task) {
  const QVariantMap body = task.body();
  const QVector3D tip = arm.jointPositions().constLast().value<QVector3D>();
  const double bend = task.hipBendDegrees() * M_PI / 180.0;
  const double yaw = task.waistYawDegrees() * M_PI / 180.0;
  const double x = tip.x();
  const double y = tip.y() + body["hipToShoulder"].toDouble();
  const double z = body["shoulderLateral"].toDouble();
  const double xb = x * std::cos(bend) + y * std::sin(bend);
  const double yb = -x * std::sin(bend) + y * std::cos(bend);
  return QVector3D(float(xb * std::cos(yaw) + z * std::sin(yaw)), float(yb + task.hipHeight()),
                   float(-xb * std::sin(yaw) + z * std::cos(yaw)));
}

} // namespace

TEST(TaskController, ObjectsStandOnTheFloorOrTheTable) {
  Rig rig;
  const QVariantMap table = rig.task.tables().first().toMap();
  const double table_x = (table["x0"].toDouble() + table["x1"].toDouble()) / 2.0;
  const double table_z = (table["z0"].toDouble() + table["z1"].toDouble()) / 2.0;

  const int on_table = rig.task.addObject("can", 0.0, table_x, table_z);
  const int on_floor = rig.task.addObject("bottle", 0.0, 0.6, 0.4);
  ASSERT_NE(on_table, -1);
  ASSERT_NE(on_floor, -1);
  EXPECT_DOUBLE_EQ(objectById(rig.task, on_table)["y"].toDouble(), table["height"].toDouble());
  EXPECT_DOUBLE_EQ(objectById(rig.task, on_floor)["y"].toDouble(), 0.0);
  // Default mass for the type.
  EXPECT_DOUBLE_EQ(objectById(rig.task, on_table)["mass"].toDouble(), 0.35);
}

TEST(TaskController, RejectsSpotsThatDontWork) {
  Rig rig;
  const QVariantMap table = rig.task.tables().first().toMap();
  EXPECT_EQ(rig.task.addObject("bottle", 1.0, 0.1, 0.1), -1);                        // on the robot's base
  EXPECT_EQ(rig.task.addObject("bottle", 1.0, table["x0"].toDouble(), -0.5), -1);   // straddling the table edge
  EXPECT_EQ(rig.task.addObject("teapot", 1.0, 0.6, 0.4), -1);                       // unknown type
  ASSERT_NE(rig.task.addObject("bottle", 1.0, 0.6, 0.4), -1);
  EXPECT_EQ(rig.task.addObject("can", 0.3, 0.61, 0.41), -1);                        // overlapping
}

TEST(TaskController, PicksUpAnObjectFromTheFloorAndTheArmFeelsItsWeight) {
  Rig rig;
  const int id = rig.task.addObject("bottle", 1.5, 0.62, 0.30);
  ASSERT_NE(id, -1);

  ASSERT_TRUE(rig.task.pickUp(id)) << rig.task.status().toStdString();
  EXPECT_TRUE(rig.task.busy());
  rig.runToEnd();

  EXPECT_FALSE(rig.task.busy());
  EXPECT_EQ(rig.task.heldId(), id);
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 1.5);
  EXPECT_FALSE(rig.arm.gripperOpen());
  EXPECT_TRUE(rig.task.status().startsWith("Holding")) << rig.task.status().toStdString();
  EXPECT_EQ(rig.hand.currentGesture(), QString("grasp"));
}

TEST(TaskController, PutsTheHeldObjectDownOnTheTable) {
  Rig rig;
  const int id = rig.task.addObject("can", 0.0, 0.62, 0.30);
  ASSERT_TRUE(rig.task.pickUp(id));
  rig.runToEnd();
  ASSERT_EQ(rig.task.heldId(), id);

  const QVariantMap table = rig.task.tables().first().toMap();
  ASSERT_TRUE(rig.task.putDown(0.40, -0.50)) << rig.task.status().toStdString();
  rig.runToEnd();

  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 0.0);
  const QVariantMap placed = objectById(rig.task, id);
  EXPECT_NEAR(placed["x"].toDouble(), 0.40, 1e-12);
  EXPECT_NEAR(placed["z"].toDouble(), -0.50, 1e-12);
  EXPECT_DOUBLE_EQ(placed["y"].toDouble(), table["height"].toDouble());
  // Back upright afterwards.
  EXPECT_NEAR(rig.task.hipBendDegrees(), 0.0, 1e-9);
}

TEST(TaskController, RefusesWhatTheJointsCantLift) {
  Rig rig;
  const int id = rig.task.addObject("dumbbell", 60.0, 0.62, 0.30);
  ASSERT_NE(id, -1);
  EXPECT_FALSE(rig.task.pickUp(id));
  EXPECT_FALSE(rig.task.busy());
  EXPECT_NE(rig.task.status().indexOf("too heavy"), -1) << rig.task.status().toStdString();
}

TEST(TaskController, ClickModesPlaceMoveAndSelect) {
  Rig rig;
  rig.task.setPlaceType("ball");
  rig.task.setClickMode("place");
  rig.task.handleClick(0.6, 0.4);
  ASSERT_EQ(rig.task.objects().size(), 1);
  EXPECT_EQ(rig.task.clickMode(), QString()); // one drop per arming
  const int id = rig.task.objects()[0].toMap()["id"].toInt();

  rig.task.setClickMode("move");
  rig.task.handleClick(0.7, 0.5);
  EXPECT_NEAR(objectById(rig.task, id)["x"].toDouble(), 0.7, 1e-12);

  rig.task.selectObject(-1);
  rig.task.handleClick(0.71, 0.5); // near the ball -> selects it
  EXPECT_EQ(rig.task.selectedId(), id);
  rig.task.handleClick(0.2, 0.9); // empty floor -> deselects
  EXPECT_EQ(rig.task.selectedId(), -1);
}

TEST(TaskController, ManualBodyControlsAreClampedToTheBodysRange) {
  Rig rig;
  rig.task.setHipHeight(5.0);
  EXPECT_DOUBLE_EQ(rig.task.hipHeight(), rig.task.body()["maxHipHeight"].toDouble());
  rig.task.setHipBendDegrees(200.0);
  EXPECT_DOUBLE_EQ(rig.task.hipBendDegrees(), rig.task.body()["maxHipBend"].toDouble());
  // The arm's torque model follows the bend.
  EXPECT_DOUBLE_EQ(rig.arm.baseTiltDegrees(), rig.task.hipBendDegrees());
}

TEST(ArmController, LeaningTheBaseIsTheSameAsTurningTheShoulder) {
  // For a planar arm, tilting the base by b within the arm's plane is
  // physically identical to the shoulder joint being b further around.
  ArmController leaning(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json");
  ArmController upright(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json");
  leaning.setPayloadKg(3.0);
  upright.setPayloadKg(3.0);

  leaning.setBaseTiltDegrees(30.0);
  leaning.setJointAngleDegrees(0, -20.0);
  leaning.setJointAngleDegrees(1, 70.0);
  upright.setJointAngleDegrees(0, -50.0); // -20 - 30
  upright.setJointAngleDegrees(1, 70.0);

  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(leaning.jointTorques()[i].toDouble(), upright.jointTorques()[i].toDouble(), 1e-9) << "joint " << i;
  }
  EXPECT_NEAR(leaning.currentMaxSafePayloadKg(), upright.currentMaxSafePayloadKg(), 1e-6);
}

TEST(TaskController, DraggingTheHandMovesTheWholeBodyThere) {
  Rig rig;
  // Low and off to the side: the waist has to turn and the hip bend.
  ASSERT_TRUE(rig.task.reachTo(0.35, 0.30, 0.55));
  EXPECT_TRUE(rig.task.reaching());
  rig.runToEnd();
  EXPECT_FALSE(rig.task.reaching());
  const QVector3D tip = tipInWorld(rig.arm, rig.task);
  EXPECT_NEAR(tip.x(), 0.35, 2e-3);
  EXPECT_NEAR(tip.y(), 0.30, 2e-3);
  EXPECT_NEAR(tip.z(), 0.55, 2e-3);
  EXPECT_GT(rig.task.hipBendDegrees(), 1.0);

  // Out of reach: says so, but still leans as far as it can toward it.
  EXPECT_FALSE(rig.task.reachTo(3.0, 1.0, 0.0));
  rig.runToEnd();
  EXPECT_GT(tipInWorld(rig.arm, rig.task).x(), 0.5);
}

TEST(TaskController, AFarReachIsAVisibleMotionNotAJump) {
  Rig rig;
  const double start_height = rig.task.hipHeight();
  ASSERT_TRUE(rig.task.reachTo(0.35, 0.30, 0.55)); // needs a big turn + crouch + lean

  // One frame later the body has only started to move, within its speeds.
  rig.task.advance(1.0 / 60.0);
  EXPECT_LT(std::abs(rig.task.waistYawDegrees()), 100.0 / 60.0 + 1e-6);
  EXPECT_LT(std::abs(rig.task.hipHeight() - start_height), 0.45 / 60.0 + 1e-6);
  EXPECT_LT(rig.task.hipBendDegrees(), 70.0 / 60.0 + 1e-6);

  // And it gets there over a fraction of a second or more, not instantly.
  int frames = 1;
  while (rig.task.reaching() && frames < 600) {
    rig.task.advance(1.0 / 60.0);
    frames++;
  }
  EXPECT_FALSE(rig.task.reaching());
  EXPECT_GT(frames, 20);
}

TEST(TaskController, AReachSpeedsUpAndSlowsDownWithinItsAccelerationLimits) {
  // Speed changes gradually, frame by frame - no instant jump to full
  // speed at the start, no instant stop at the end (the joints' torque
  // would have to be unbounded for either).
  Rig rig;
  ASSERT_TRUE(rig.task.reachTo(0.35, 0.30, 0.55));
  const double dt = 1.0 / 60.0;
  double yaw = rig.task.waistYawDegrees() * M_PI / 180.0;
  double lift = rig.task.hipHeight();
  double shoulder = rig.arm.jointAnglesRadians()[0];
  double yaw_speed = 0.0;
  double lift_speed = 0.0;
  double shoulder_speed = 0.0;
  for (int frame = 0; frame < 600 && rig.task.reaching(); frame++) {
    rig.task.advance(dt);
    if (!rig.task.reaching()) {
      break; // the last frame snaps the final millimetre
    }
    const double new_yaw = rig.task.waistYawDegrees() * M_PI / 180.0;
    const double new_lift = rig.task.hipHeight();
    const double new_shoulder = rig.arm.jointAnglesRadians()[0];
    const double new_yaw_speed = (new_yaw - yaw) / dt;
    const double new_lift_speed = (new_lift - lift) / dt;
    const double new_shoulder_speed = (new_shoulder - shoulder) / dt;
    EXPECT_LE(std::abs(new_yaw_speed - yaw_speed), 6.0 * dt + 1e-6) << "frame " << frame;
    EXPECT_LE(std::abs(new_lift_speed - lift_speed), 2.0 * dt + 1e-6) << "frame " << frame;
    EXPECT_LE(std::abs(new_shoulder_speed - shoulder_speed), 10.0 * dt + 1e-6) << "frame " << frame;
    yaw = new_yaw;
    lift = new_lift;
    shoulder = new_shoulder;
    yaw_speed = new_yaw_speed;
    lift_speed = new_lift_speed;
    shoulder_speed = new_shoulder_speed;
  }
  EXPECT_FALSE(rig.task.reaching());
  EXPECT_LT(std::abs(yaw_speed), 0.2); // nearly stopped by the time it arrives
}

TEST(TaskController, DynamicModeFeelsThePickUpBeingMade) {
  // While the robot moves, the arm's Dynamic-mode loads are those of the
  // motion it is actually making: the joint rates it reports match how
  // its angles change frame to frame, and the inertia part of the load is
  // non-zero. Once it stops, everything is back at rest.
  Rig rig;
  rig.arm.setDynamicMode(true);
  const int id = rig.task.addObject("bottle", 1.5, 0.62, 0.30);
  ASSERT_TRUE(rig.task.pickUp(id)) << rig.task.status().toStdString();

  const double dt = 1.0 / 60.0;
  double peak_speed = 0.0;
  double peak_inertia = 0.0;
  for (int frame = 0; frame < 60 * 30 && rig.task.busy(); frame++) {
    const std::vector<double> before = rig.arm.jointAnglesRadians();
    rig.task.advance(dt);
    const std::vector<double> after = rig.arm.jointAnglesRadians();
    const QVariantList speeds = rig.arm.jointVelocitiesDegrees();
    for (size_t i = 0; i < after.size(); i++) {
      const double reported = speeds[static_cast<int>(i)].toDouble() * M_PI / 180.0;
      const double measured = (after[i] - before[i]) / dt;
      EXPECT_NEAR(reported, measured, 0.1) << "joint " << i << ", frame " << frame;
      peak_speed = std::max(peak_speed, std::abs(reported));
      peak_inertia = std::max(peak_inertia, std::abs(rig.arm.inertiaTorques()[static_cast<int>(i)].toDouble()));
    }
  }
  EXPECT_FALSE(rig.task.busy());
  EXPECT_GT(peak_speed, 0.3);
  EXPECT_GT(peak_inertia, 0.05);
  for (const QVariant &speed : rig.arm.jointVelocitiesDegrees()) {
    EXPECT_DOUBLE_EQ(speed.toDouble(), 0.0);
  }
  for (const QVariant &torque : rig.arm.inertiaTorques()) {
    EXPECT_NEAR(torque.toDouble(), 0.0, 1e-12);
  }
}

TEST(TaskController, TheBodyCrouchingLoadsTheArm) {
  // Lowering the lift and leaning accelerate the shoulder: the arm's
  // dynamics see its base move, and only while the body moves.
  Rig rig;
  rig.arm.setDynamicMode(true);
  const double start = rig.task.hipHeight();
  ASSERT_TRUE(rig.task.reachTo(0.35, 0.30, 0.55)); // a big turn + crouch + lean
  double peak_vertical = 0.0;
  double peak_turn = 0.0;
  for (int frame = 0; frame < 600 && rig.task.reaching(); frame++) {
    rig.task.advance(1.0 / 60.0);
    peak_vertical = std::max(peak_vertical, std::abs(rig.arm.baseMotion().linear_acceleration.y()));
    peak_turn = std::max(peak_turn, rig.arm.baseMotion().angular_velocity.norm());
  }
  EXPECT_LT(rig.task.hipHeight(), start - 0.04);
  EXPECT_GT(peak_vertical, 0.3);
  EXPECT_GT(peak_turn, 0.3);
  EXPECT_NEAR(rig.arm.baseMotion().linear_acceleration.norm(), 0.0, 1e-12);
  EXPECT_NEAR(rig.arm.baseMotion().angular_velocity.norm(), 0.0, 1e-12);
}

TEST(TaskController, TouchingTheBodyCancelsTheReach) {
  Rig rig;
  ASSERT_TRUE(rig.task.reachTo(0.35, 0.30, 0.55));
  rig.task.advance(0.1);
  rig.task.setHipHeight(0.9);
  EXPECT_FALSE(rig.task.reaching());
  EXPECT_DOUBLE_EQ(rig.task.hipHeight(), 0.9);
}

TEST(TaskController, IgnoresDragsWhileAMoveIsPlaying) {
  Rig rig;
  const int id = rig.task.addObject("can", 0.0, 0.62, 0.30);
  ASSERT_TRUE(rig.task.pickUp(id));
  const double yaw = rig.task.waistYawDegrees();
  EXPECT_FALSE(rig.task.reachTo(0.4, 1.0, -0.4));
  EXPECT_DOUBLE_EQ(rig.task.waistYawDegrees(), yaw);
}

TEST(TaskController, WarehouseArmPicksWithItsGripper) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  EXPECT_EQ(rig.arm.armStyle(), QString("industrial"));
  rig.arm.addJoint(); // four joints now - no closed form
  ASSERT_EQ(rig.arm.jointCount(), 4);

  const int id = rig.task.addObject("can", 0.0, 0.55, 0.35);
  ASSERT_TRUE(rig.task.pickUp(id)) << rig.task.status().toStdString();
  rig.runToEnd();
  ASSERT_EQ(rig.task.heldId(), id);
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 0.35);
  // Closed on the can, and it says how it's held.
  EXPECT_LT(rig.task.gripperWidth(), 0.08);
  const QString mode = rig.task.heldObject()["graspMode"].toString();
  EXPECT_TRUE(mode == "top" || mode == "side") << mode.toStdString();

  ASSERT_TRUE(rig.task.putDown(0.40, -0.50)) << rig.task.status().toStdString();
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_NEAR(rig.task.gripperWidth(), 0.11, 1e-12);
  EXPECT_NEAR(objectById(rig.task, id)["x"].toDouble(), 0.40, 1e-12);
}

TEST(TaskController, SwitchesBetweenTheHumanoidAndTheWarehouseArm) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  rig.arm.addJoint();
  rig.arm.addJoint();
  ASSERT_EQ(rig.arm.jointCount(), 5);

  ASSERT_TRUE(rig.task.useHumanoidArm());
  EXPECT_EQ(rig.arm.armStyle(), QString("humanoid"));
  EXPECT_EQ(rig.arm.jointCount(), 3);
  EXPECT_EQ(rig.arm.jointNames().first(), QString("Shoulder"));

  // Not while holding something.
  const int id = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  ASSERT_TRUE(rig.task.pickUp(id));
  rig.runToEnd();
  EXPECT_FALSE(rig.task.useIndustrialArm());
  EXPECT_EQ(rig.arm.armStyle(), QString("humanoid"));
}

TEST(TaskController, TablesCanBeAddedMovedAndRemoved) {
  Rig rig;
  ASSERT_EQ(rig.task.tables().size(), 1);

  // A coffee table on the robot's right; a can on it.
  const int coffee = rig.task.addTable("coffee table", 0.55, 0.60);
  ASSERT_NE(coffee, -1) << rig.task.status().toStdString();
  const int can = rig.task.addObject("can", 0.0, 0.55, 0.60);
  ASSERT_NE(can, -1);
  EXPECT_NEAR(objectById(rig.task, can)["y"].toDouble(), 0.42, 1e-12);

  // Moving the table carries the can along.
  ASSERT_TRUE(rig.task.moveTable(coffee, 0.70, 0.70));
  EXPECT_NEAR(objectById(rig.task, can)["x"].toDouble(), 0.70, 1e-12);
  EXPECT_NEAR(objectById(rig.task, can)["z"].toDouble(), 0.70, 1e-12);

  // Not into the robot, not onto the other table.
  EXPECT_FALSE(rig.task.moveTable(coffee, 0.20, 0.10));
  EXPECT_FALSE(rig.task.moveTable(coffee, 0.50, -0.55));
  EXPECT_EQ(rig.task.addTable("counter", 0.0, 0.0), -1);

  // Removing it sets the can down on the floor.
  rig.task.removeTable(coffee);
  EXPECT_EQ(rig.task.tables().size(), 1);
  EXPECT_DOUBLE_EQ(objectById(rig.task, can)["y"].toDouble(), 0.0);
}

TEST(TaskController, PutsThingsDownOnAnAddedCounter) {
  Rig rig;
  const int counter = rig.task.addTable("counter", 0.60, 0.55);
  ASSERT_NE(counter, -1) << rig.task.status().toStdString();
  const int id = rig.task.addObject("bottle", 0.0, 0.62, -0.10);
  ASSERT_TRUE(rig.task.pickUp(id)) << rig.task.status().toStdString();
  rig.runToEnd();
  ASSERT_TRUE(rig.task.putDown(0.55, 0.45)) << rig.task.status().toStdString();
  rig.runToEnd();
  EXPECT_NEAR(objectById(rig.task, id)["y"].toDouble(), 0.92, 1e-12);
}

TEST(TaskController, EveryObjectTypeCanBePickedUpAndPutDown) {
  for (const QString &type : Rig().task.objectTypes()) {
    Rig rig;
    const int id = rig.task.addObject(type, 0.0, 0.62, 0.30);
    ASSERT_NE(id, -1) << type.toStdString();
    ASSERT_TRUE(rig.task.pickUp(id)) << type.toStdString() << ": " << rig.task.status().toStdString();
    rig.runToEnd();
    EXPECT_EQ(rig.task.heldId(), id) << type.toStdString();
    ASSERT_TRUE(rig.task.putDown(0.45, -0.55)) << type.toStdString() << ": " << rig.task.status().toStdString();
    rig.runToEnd();
    EXPECT_EQ(rig.task.heldId(), -1) << type.toStdString();
  }
}

TEST(TaskController, TheWarehouseGripperHandlesEveryObjectType) {
  for (const QString &type : Rig().task.objectTypes()) {
    Rig rig;
    ASSERT_TRUE(rig.task.useIndustrialArm());
    rig.arm.addJoint();
    // 1 kg each: the shapes are what's tested here; the added joint's
    // small motor can't hold an 8 kg dumbbell out at the gripper.
    const int id = rig.task.addObject(type, 1.0, 0.55, 0.35);
    ASSERT_NE(id, -1) << type.toStdString();
    ASSERT_TRUE(rig.task.pickUp(id)) << type.toStdString() << ": " << rig.task.status().toStdString();
    rig.runToEnd();
    EXPECT_EQ(rig.task.heldId(), id) << type.toStdString();
    ASSERT_TRUE(rig.task.putDown(0.45, -0.55)) << type.toStdString() << ": " << rig.task.status().toStdString();
    rig.runToEnd();
    EXPECT_EQ(rig.task.heldId(), -1) << type.toStdString();
  }
}

TEST(TaskController, GripClosesTheHandRightAway) {
  Rig rig;
  rig.task.setGrip(1.0);
  EXPECT_DOUBLE_EQ(rig.task.grip(), 1.0);
  EXPECT_FALSE(rig.hand.animating()); // instant, not an animation
  const double closed = rig.hand.jointAngleDegrees("index", 1);
  rig.task.setGrip(0.0);
  const double open = rig.hand.jointAngleDegrees("index", 1);
  EXPECT_GT(std::abs(closed - open), 20.0); // the finger really moved

  rig.task.setGrip(7.0); // clamped
  EXPECT_DOUBLE_EQ(rig.task.grip(), 1.0);
}

TEST(TaskController, GripClosesTheWarehouseGripper) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  rig.task.setGrip(0.0);
  EXPECT_NEAR(rig.task.gripperWidth(), 0.11, 1e-12);
  rig.task.setGrip(1.0);
  EXPECT_NEAR(rig.task.gripperWidth(), 0.01, 1e-12);
  rig.task.setGrip(0.5);
  EXPECT_NEAR(rig.task.gripperWidth(), 0.06, 1e-12);
}

TEST(TaskController, GripNeverSqueezesThroughAHeldObject) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  const int can = rig.task.addObject("can", 0.0, 0.55, 0.35);
  ASSERT_TRUE(rig.task.pickUp(can));
  rig.runToEnd();
  ASSERT_EQ(rig.task.heldId(), can);
  const double held_width = rig.task.gripperWidth();
  rig.task.setGrip(1.0);
  EXPECT_DOUBLE_EQ(rig.task.gripperWidth(), held_width);
}

TEST(TaskController, GripIsIgnoredDuringAPickUp) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  ASSERT_TRUE(rig.task.pickUp(bottle));
  rig.task.setGrip(1.0);
  EXPECT_DOUBLE_EQ(rig.task.grip(), 0.0);
}

// Brings the hand to an object's grip center the way the webcam would:
// by reaching there and letting the body move.
void handTo(Rig &rig, double x, double y, double z) {
  rig.task.reachTo(x, y, z);
  rig.runToEnd();
}

TEST(TaskController, ClosingTheHandAroundAnObjectGrabsIt) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30); // grip center 10 cm up
  handTo(rig, 0.62, 0.10, 0.30);
  rig.task.setGrip(0.5);
  EXPECT_EQ(rig.task.heldId(), -1); // half-closed: not yet
  rig.task.setGrip(0.9);
  EXPECT_EQ(rig.task.heldId(), bottle);
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 1.0); // its mass is now on the arm
}

TEST(TaskController, ClosingTheHandOnNothingGrabsNothing) {
  Rig rig;
  rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.45, 0.60, 0.30); // half a meter above it
  rig.task.setGrip(1.0);
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_TRUE(rig.task.status().contains("Nothing"));
}

TEST(TaskController, OpeningTheHandSetsTheObjectDownBelowIt) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.62, 0.10, 0.30);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), bottle);

  // Carry it over the table and open the hand.
  handTo(rig, 0.50, 0.85, -0.55);
  rig.task.setGrip(0.5);
  EXPECT_EQ(rig.task.heldId(), bottle); // between the thresholds: still held
  rig.task.setGrip(0.1);
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 0.0);
  const QVariantMap placed = objectById(rig.task, bottle);
  EXPECT_NEAR(placed["y"].toDouble(), 0.72, 1e-12); // standing on the table
  EXPECT_NEAR(placed["x"].toDouble(), 0.50, 0.08);   // right below the hand
  EXPECT_NEAR(placed["z"].toDouble(), -0.55, 0.08);
}

TEST(TaskController, KeepsHoldingOverABadSpotAndLetsGoOnceItsGood) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  rig.task.addObject("box", 0.0, 0.70, 0.10); // in the way, below where the hand goes next
  handTo(rig, 0.62, 0.10, 0.30);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), bottle);

  // Right over the box: there's no room to put the bottle down.
  handTo(rig, 0.70, 0.45, 0.10);
  rig.task.setGrip(0.0);
  EXPECT_EQ(rig.task.heldId(), bottle);
  EXPECT_TRUE(rig.task.status().contains("Can't let go")) << rig.task.status().toStdString();

  // Hand still open, moved over clear floor: it lets go on the next update.
  handTo(rig, 0.60, 0.40, -0.20);
  rig.task.setGrip(0.0);
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_DOUBLE_EQ(objectById(rig.task, bottle)["y"].toDouble(), 0.0);
}

TEST(TaskController, TheHandTwistsAtAForearmsSpeedAndStaysTwisted) {
  Rig rig;
  rig.task.setHandRoll(1.2);
  rig.task.advance(1.0 / 60.0);
  EXPECT_GT(rig.task.handRollDegrees(), 0.0);
  EXPECT_LT(rig.task.handRollDegrees(), 3.5 * 180.0 / M_PI / 60.0 + 1e-6); // not all at once (3.5 rad/s)
  rig.runToEnd();
  EXPECT_NEAR(rig.task.handRollDegrees(), 1.2 * 180.0 / M_PI, 0.1);

  // Moving the hand somewhere else keeps the twist.
  handTo(rig, 0.45, 1.0, 0.30);
  EXPECT_NEAR(rig.task.handRollDegrees(), 1.2 * 180.0 / M_PI, 0.1);
}

TEST(TaskController, TheTwistStopsAtPalmUpOrDown) {
  Rig rig;
  rig.task.setHandRoll(3.0);
  rig.runToEnd();
  EXPECT_NEAR(rig.task.handRollDegrees(), 90.0, 1e-6);
}

TEST(TaskController, APickUpTurnsTheHandBackToNeutral) {
  Rig rig;
  rig.task.setHandRoll(1.0);
  rig.runToEnd();
  const int bottle = rig.task.addObject("bottle", 0.0, 0.55, 0.30);
  ASSERT_TRUE(rig.task.pickUp(bottle)) << rig.task.status().toStdString();
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), bottle);
  EXPECT_NEAR(rig.task.handRollDegrees(), 0.0, 1e-6);
}

TEST(TaskController, TheWarehouseGripperDoesntTwist) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  rig.task.setHandRoll(1.0);
  rig.runToEnd();
  EXPECT_DOUBLE_EQ(rig.task.handRollDegrees(), 0.0);
}

TEST(TaskController, TheWarehouseGripperGrabsAndReleasesToo) {
  Rig rig;
  ASSERT_TRUE(rig.task.useIndustrialArm());
  rig.arm.addJoint();
  const int can = rig.task.addObject("can", 0.0, 0.55, 0.35); // grip center ~6 cm up
  // The gripper's grip point is 7.5 cm past the arm's tip: aim the tip short.
  handTo(rig, 0.55, 0.06 + 0.075, 0.35);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), can) << rig.task.status().toStdString();
  EXPECT_NEAR(rig.task.gripperWidth(), 0.066, 0.01); // closed on the can, not shut
  handTo(rig, 0.70, 0.40, 0.0);
  rig.task.setGrip(0.0);
  EXPECT_EQ(rig.task.heldId(), -1);
}

// ---- the assist (live control) ---------------------------------------------

namespace {

constexpr double kLiveFrame = 1.0 / 30.0;

// A webcam user's hand moving the arm's tip in a straight line from `from`
// to `to` at `speed` m/s, then holding there for `rest` s: one live update
// per camera frame, the body moving in between.
void liveMove(Rig &rig, QVector3D from, QVector3D to, double speed = 0.3, double rest = 0.5) {
  const double length = (to - from).length();
  const int moving = std::max(1, int(std::ceil(length / (speed * kLiveFrame))));
  for (int i = 1; i <= moving + int(rest / kLiveFrame); i++) {
    const QVector3D at = from + (to - from) * float(std::min(1.0, double(i) / moving));
    rig.task.guidedReach(at.x(), at.y(), at.z(), std::nan(""), kLiveFrame);
    for (int k = 0; k < 2; k++) {
      rig.task.advance(kLiveFrame / 2.0);
    }
  }
}

const QVector3D kLiveStart(0.40, 0.65, 0.25);

} // namespace

TEST(TaskController, AssistGuessesTheObjectTheHandIsGoingFor) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  const int box = rig.task.addObject("box", 0.0, 0.62, -0.10);
  liveMove(rig, kLiveStart, QVector3D(0.55, 0.12, 0.30));
  EXPECT_EQ(rig.task.intentId(), bottle);
  liveMove(rig, QVector3D(0.55, 0.12, 0.30), QVector3D(0.58, 0.10, -0.05));
  EXPECT_EQ(rig.task.intentId(), box); // changed its mind with the hand
}

TEST(TaskController, AssistPullsAnAlmostRightHandOntoTheObject) {
  // Aimed 13 cm short and 5 cm to the side - what a webcam's depth guess
  // does - closing the hand still grabs.
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  liveMove(rig, kLiveStart, QVector3D(0.49, 0.10, 0.35));
  EXPECT_EQ(rig.task.intentId(), bottle);
  EXPECT_GT(rig.task.assistStrength(), 0.4);
  rig.task.setGrip(1.0);
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), bottle) << rig.task.status().toStdString();
}

TEST(TaskController, WithoutTheAssistTheSameMissMisses) {
  Rig rig;
  rig.task.setAssist(false);
  rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  liveMove(rig, kLiveStart, QVector3D(0.49, 0.10, 0.35));
  EXPECT_EQ(rig.task.intentId(), -1);
  rig.task.setGrip(1.0);
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), -1);
}

TEST(TaskController, AssistFinishesTheGrabFromAFewCentimetresOff) {
  // Too far for the pull alone: closing the hand sends it the rest of the
  // way, and it grabs on arrival.
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  liveMove(rig, kLiveStart, QVector3D(0.47, 0.16, 0.37));
  ASSERT_EQ(rig.task.intentId(), bottle);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), -1); // not around it yet
  EXPECT_TRUE(rig.task.status().contains("Taking")) << rig.task.status().toStdString();
  // Live updates keep coming while it does: they don't take over.
  EXPECT_FALSE(rig.task.guidedReach(0.47, 0.16, 0.37, std::nan(""), kLiveFrame));
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), bottle) << rig.task.status().toStdString();
}

TEST(TaskController, OpeningTheHandStopsTheAssistsGrab) {
  Rig rig;
  rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  liveMove(rig, kLiveStart, QVector3D(0.47, 0.16, 0.37));
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), -1);
  ASSERT_TRUE(rig.task.reaching()); // on its way to grab
  rig.task.advance(1.0 / 60.0);
  rig.task.setGrip(0.0);
  EXPECT_FALSE(rig.task.reaching());
  rig.runToEnd();
  EXPECT_EQ(rig.task.heldId(), -1);
}

TEST(TaskController, MouseDragsAreNotPulled) {
  // The hand goes exactly where it's dragged (the object in reach is only
  // ringed - see grabOrRelease).
  Rig rig;
  rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.49, 0.10, 0.35);
  EXPECT_EQ(rig.task.assistStrength(), 0.0);
  const QVector3D tip = tipInWorld(rig.arm, rig.task);
  EXPECT_NEAR(tip.x(), 0.49, 0.01);
  EXPECT_NEAR(tip.y(), 0.10, 0.01);
  EXPECT_NEAR(tip.z(), 0.35, 0.01);
}

TEST(TaskController, HoldingSomethingTheAssistRests) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  rig.task.addObject("box", 0.0, 0.62, -0.10);
  handTo(rig, 0.62, 0.10, 0.30);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), bottle);
  liveMove(rig, QVector3D(0.62, 0.30, 0.30), QVector3D(0.58, 0.30, -0.08));
  EXPECT_EQ(rig.task.intentId(), -1);
}

// ---- the mouse's grab ------------------------------------------------------

TEST(TaskController, DraggingTheHandNearAnObjectRingsIt) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.45, 0.60, 0.30); // far above it
  EXPECT_EQ(rig.task.intentId(), -1);
  handTo(rig, 0.52, 0.12, 0.33); // a hand's width short of it
  EXPECT_EQ(rig.task.intentId(), bottle);
  EXPECT_GT(rig.task.intentConfidence(), 0.0);
}

TEST(TaskController, ClickingTheHandNearAnObjectGrabsItAndAgainLetsGo) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.50, 0.14, 0.34); // close, but not around it
  ASSERT_TRUE(rig.task.grabOrRelease()) << rig.task.status().toStdString();
  rig.runToEnd();
  ASSERT_EQ(rig.task.heldId(), bottle) << rig.task.status().toStdString();

  handTo(rig, 0.60, 0.40, -0.20); // over clear floor
  EXPECT_TRUE(rig.task.grabOrRelease());
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_DOUBLE_EQ(objectById(rig.task, bottle)["y"].toDouble(), 0.0);
}

TEST(TaskController, ClickingTheHandAroundAnObjectGrabsItRightAway) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.62, 0.10, 0.30);
  EXPECT_TRUE(rig.task.grabOrRelease());
  EXPECT_EQ(rig.task.heldId(), bottle);
}

TEST(TaskController, ClickingTheHandFarFromEverythingSaysSo) {
  Rig rig;
  rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  handTo(rig, 0.45, 0.60, 0.30);
  EXPECT_FALSE(rig.task.grabOrRelease());
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_TRUE(rig.task.status().contains("Nothing close enough")) << rig.task.status().toStdString();
}

// ---- the left arm ----------------------------------------------------------

namespace {

struct TwoArmRig : Rig {
  ArmController left{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController leftHand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  TwoArmRig() { task.setLeftArm(&left, &leftHand); }
};

// Where the left arm's end-effector point is (the mirrored robot's right).
Eigen::Vector3d leftTip(const TwoArmRig &rig) {
  robotarm::BodyPose pose;
  pose.yaw = rig.task.waistYawDegrees() * M_PI / 180.0;
  pose.hip_height = rig.task.hipHeight();
  pose.hip_bend = rig.task.hipBendDegrees() * M_PI / 180.0;
  pose.arm = rig.left.jointAnglesRadians();
  const robotarm::BodyGeometry body;
  return robotarm::mirrored(
      robotarm::grip_point_in_world(rig.left.config(), body, robotarm::mirrored(pose), 0.0, 0.0));
}

void leftTo(TwoArmRig &rig, double x, double y, double z, bool leads = true) {
  rig.task.reachLeft(x, y, z, std::nan(""), leads);
  rig.runToEnd();
}

} // namespace

TEST(TaskController, WithoutALeftArmLeftCommandsDoNothing) {
  Rig rig;
  EXPECT_EQ(rig.task.leftArm(), nullptr);
  EXPECT_FALSE(rig.task.reachLeft(0.4, 1.0, -0.3, std::nan(""), true));
  rig.task.setLeftGrip(1.0);
  EXPECT_EQ(rig.task.leftHeldId(), -1);
}

TEST(TaskController, TheLeftArmLeadingTurnsTheBodyToReach) {
  TwoArmRig rig;
  const Eigen::Vector3d point(0.35, 0.95, -0.45); // ahead and to the left
  leftTo(rig, point.x(), point.y(), point.z());
  EXPECT_LT((leftTip(rig) - point).norm(), 0.01);
  EXPECT_GT(rig.task.waistYawDegrees(), 1.0); // turned left (counter-clockwise from above)
}

TEST(TaskController, WithTheRightArmLeadingTheLeftReachesWithItsJointsAlone) {
  TwoArmRig rig;
  const double yaw = rig.task.waistYawDegrees();
  const double hip = rig.task.hipHeight();
  leftTo(rig, 0.40, 1.05, -0.45, false); // 25 cm off its plane: only the in-plane part
  EXPECT_DOUBLE_EQ(rig.task.waistYawDegrees(), yaw);
  EXPECT_DOUBLE_EQ(rig.task.hipHeight(), hip);
  const Eigen::Vector3d tip = leftTip(rig);
  EXPECT_NEAR(tip.x(), 0.40, 0.01);
  EXPECT_NEAR(tip.y(), 1.05, 0.01);
  EXPECT_NEAR(tip.z(), -0.20, 0.01); // the left shoulder's plane
}

TEST(TaskController, TheHandThatMovedLastSteersTheBody) {
  TwoArmRig rig;
  const Eigen::Vector3d left_target(0.35, 0.95, -0.45); // 25 cm off the left arm's plane
  const auto frame = [&](const QVector3D &right, const Eigen::Vector3d &left) {
    rig.task.guidedReach(right.x(), right.y(), right.z(), std::nan(""), 1.0 / 30.0);
    rig.task.reachLeft(left.x(), left.y(), left.z(), std::nan(""));
    rig.task.advance(1.0 / 30.0);
  };
  // The right hand moving, the left held still: the right steers, the
  // left can only reach with its joints - not out to its target.
  for (int i = 0; i <= 15; i++) {
    frame(QVector3D(0.45, 1.0 - 0.01 * i, 0.30 + 0.01 * i), left_target);
  }
  rig.runToEnd();
  EXPECT_LT((tipInWorld(rig.arm, rig.task) - QVector3D(0.45, 0.85, 0.45)).length(), 0.01);
  EXPECT_GT((leftTip(rig) - left_target).norm(), 0.1);

  // Now the left hand moves and the right is still: the left steers, and
  // gets there.
  const Eigen::Vector3d left_goal(0.40, 1.0, -0.50);
  for (int i = 0; i <= 15; i++) {
    frame(QVector3D(0.45, 0.85, 0.45), left_target + (left_goal - left_target) * (i / 15.0));
  }
  rig.runToEnd();
  EXPECT_LT((leftTip(rig) - left_goal).norm(), 0.01);
}

TEST(TaskController, TheLeftHandGrabsCarriesAndLetsGo) {
  TwoArmRig rig;
  // On the table, on the robot's left: grip center 72 + 10 cm up.
  const int bottle = rig.task.addObject("bottle", 0.0, 0.50, -0.45);
  ASSERT_DOUBLE_EQ(objectById(rig.task, bottle)["y"].toDouble(), 0.72);
  leftTo(rig, 0.50, 0.82, -0.45);
  rig.task.setLeftGrip(1.0);
  ASSERT_EQ(rig.task.leftHeldId(), bottle) << rig.task.status().toStdString();
  EXPECT_EQ(rig.task.heldId(), -1);
  EXPECT_DOUBLE_EQ(rig.left.payloadKg(), 1.0); // on the left arm's joints
  EXPECT_DOUBLE_EQ(rig.arm.payloadKg(), 0.0);
  EXPECT_TRUE(objectById(rig.task, bottle)["held"].toBool());
  EXPECT_EQ(rig.task.leftHeldObject()["id"].toInt(), bottle);

  // The right hand can't take it, nor can a pick-up.
  EXPECT_FALSE(rig.task.pickUp(bottle));

  leftTo(rig, 0.55, 0.40, 0.10); // over clear floor
  rig.task.setLeftGrip(0.0);
  EXPECT_EQ(rig.task.leftHeldId(), -1);
  EXPECT_DOUBLE_EQ(rig.left.payloadKg(), 0.0);
  EXPECT_DOUBLE_EQ(objectById(rig.task, bottle)["y"].toDouble(), 0.0);
}

TEST(TaskController, TheLeftHandMovesInToGrabFromAFewCentimetresOff) {
  TwoArmRig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.50, -0.45); // on the table
  leftTo(rig, 0.42, 0.88, -0.50);
  rig.task.setLeftGrip(1.0);
  ASSERT_EQ(rig.task.leftHeldId(), -1); // not around it yet
  EXPECT_TRUE(rig.task.status().contains("taking")) << rig.task.status().toStdString();
  rig.runToEnd();
  for (int i = 0; i < 600 && rig.task.leftHeldId() == -1; i++) {
    rig.task.advance(1.0 / 60.0);
  }
  EXPECT_EQ(rig.task.leftHeldId(), bottle) << rig.task.status().toStdString();
}

TEST(TaskController, BothHandsHoldSomethingAtOnce) {
  TwoArmRig rig;
  const int right = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  const int left = rig.task.addObject("can", 0.0, 0.50, -0.45); // on the table: grip center at 0.78
  handTo(rig, 0.62, 0.10, 0.30);
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), right);
  leftTo(rig, 0.50, 0.78, -0.45); // the body turns for it (the right hand keeps the bottle)
  rig.task.setLeftGrip(1.0);
  for (int i = 0; i < 600 && rig.task.leftHeldId() == -1; i++) {
    rig.task.advance(1.0 / 60.0);
  }
  EXPECT_EQ(rig.task.leftHeldId(), left) << rig.task.status().toStdString();
  EXPECT_EQ(rig.task.heldId(), right);
}

TEST(TaskController, TheLeftHandTwistsAndLeansWithTheBody) {
  TwoArmRig rig;
  rig.task.setLeftHandRoll(0.8);
  rig.runToEnd();
  for (int i = 0; i < 120; i++) {
    rig.task.advance(1.0 / 60.0);
  }
  EXPECT_NEAR(rig.task.leftHandRollDegrees(), 0.8 * 180 / M_PI, 0.5);
  rig.task.setHipBendDegrees(20.0);
  EXPECT_DOUBLE_EQ(rig.left.baseTiltDegrees(), 20.0); // one torso: both arms lean
}
