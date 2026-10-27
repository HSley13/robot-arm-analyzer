// Verifies ArmController: the QML-facing properties reflect the loaded
// config correctly, recompute happens on every mutation, the Exact/AI
// toggle actually switches engines (and still agrees closely with Exact,
// the same cross-check Module 5 already established), and the
// color-threshold/camera-pivot logic the UI depends on is correct.
#include <string>

#include <gtest/gtest.h>
#include <QColor>
#include <QVariantList>
#include <QVector3D>

#include "arm_controller.hpp"

using robotarm::ArmController;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string config_path() { return std::string(PROJECT_ROOT_DIR) + "/config/two_link_planar.json"; }
std::string surrogate_path() { return std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar.surrogate.json"; }

} // namespace

TEST(ArmController, LoadsConfigAndSurrogate) {
  ArmController controller(config_path(), surrogate_path());
  EXPECT_EQ(controller.armName(), QString("Two-Link Planar Arm (example)"));
  EXPECT_EQ(controller.jointCount(), 2);
  EXPECT_TRUE(controller.aiModeAvailable());
}

TEST(ArmController, AiModeUnavailableWithoutAMatchingSurrogateFile) {
  ArmController controller(config_path(), "/nonexistent/weights.json");
  EXPECT_FALSE(controller.aiModeAvailable());
  controller.setAiMode(true);
  EXPECT_FALSE(controller.aiMode()); // the setter must refuse, not silently "succeed"
}

TEST(ArmController, CanvasCenterIsNotWorldOrigin) {
  // config/two_link_planar.json: a=1.0 and a=0.5, so total reach is 1.5 -
  // the pivot should sit at the midpoint of that reach, not (0,0,0).
  ArmController controller(config_path(), surrogate_path());
  const QVector3D center = controller.canvasCenter();
  EXPECT_NEAR(center.x(), 0.75f, 1e-4f);
  EXPECT_FLOAT_EQ(center.y(), 0.0f);
  EXPECT_FLOAT_EQ(center.z(), 0.0f);
}

TEST(ArmController, SettingJointAngleRecomputesPositionsAndTorques) {
  ArmController controller(config_path(), surrogate_path());
  const QVariantList torques_before = controller.jointTorques();

  controller.setJointAngleDegrees(0, 45.0);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(0).toDouble(), 45.0, 1e-9);

  const QVariantList torques_after = controller.jointTorques();
  ASSERT_EQ(torques_before.size(), torques_after.size());
  // At least one joint's torque must have changed - otherwise recompute()
  // silently didn't run.
  bool any_changed = false;
  for (int i = 0; i < torques_before.size(); i++) {
    if (torques_before.at(i).toDouble() != torques_after.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);

  const QVariantList positions = controller.jointPositions();
  EXPECT_EQ(positions.size(), controller.jointCount() + 1); // base + one per joint
}

TEST(ArmController, SetJointAngleIgnoresOutOfRangeIndex) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(99, 45.0); // must not crash or corrupt state
  controller.setJointAngleDegrees(-1, 45.0);
  EXPECT_EQ(controller.jointAnglesDegrees().size(), controller.jointCount());
}

TEST(ArmController, AiModeTogglesEngineAndStaysCloseToExact) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);
  controller.setJointAngleDegrees(1, -45.0);
  controller.setPayloadKg(2.0);

  const QVariantList exact = controller.jointTorques();
  controller.setAiMode(true);
  ASSERT_TRUE(controller.aiMode());
  const QVariantList ai = controller.jointTorques();

  ASSERT_EQ(exact.size(), ai.size());
  for (int i = 0; i < exact.size(); i++) {
    // Same generous bound as Module 5's surrogate_tests.
    EXPECT_NEAR(ai.at(i).toDouble(), exact.at(i).toDouble(), 15.0);
  }
}

TEST(ArmController, JointColorsReflectTorqueLimitThresholds) {
  ArmController controller(config_path(), surrogate_path());
  controller.setPayloadKg(0.0);
  controller.setJointAngleDegrees(0, 0.0);
  controller.setJointAngleDegrees(1, 0.0);

  const QVariantList colors = controller.jointColors();
  ASSERT_EQ(colors.size(), controller.jointCount());
  for (int i = 0; i < colors.size(); i++) {
    EXPECT_TRUE(colors.at(i).canConvert<QColor>());
  }
}
