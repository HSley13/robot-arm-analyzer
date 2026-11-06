#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
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
    for (int i = 0; i < 60 * 30 && task.busy(); i++) {
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

} // namespace

TEST(TaskController, ObjectsStandOnTheFloorOrTheTable) {
  Rig rig;
  const QVariantMap table = rig.task.table();
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
  const QVariantMap table = rig.task.table();
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

  const QVariantMap table = rig.task.table();
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
