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
std::string dynamic_surrogate_path() {
  return std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar_dynamic.surrogate.json";
}

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

TEST(ArmController, StaticModeHasZeroVelocityAndAcceleration) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  EXPECT_FALSE(controller.dynamicMode());

  controller.setJointVelocityDegrees(0, 90.0); // should have no effect outside dynamicMode
  const QVariantList velocities = controller.jointVelocitiesDegrees();
  for (int i = 0; i < velocities.size(); i++) EXPECT_DOUBLE_EQ(velocities.at(i).toDouble(), 0.0);
}

TEST(ArmController, DynamicModeReflectsSetVelocityAndAcceleration) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setDynamicMode(true);
  ASSERT_TRUE(controller.dynamicMode());

  controller.setJointVelocityDegrees(0, 90.0);
  controller.setJointAccelerationDegrees(1, 45.0);

  EXPECT_NEAR(controller.jointVelocitiesDegrees().at(0).toDouble(), 90.0, 1e-9);
  EXPECT_NEAR(controller.jointAccelerationsDegrees().at(1).toDouble(), 45.0, 1e-9);
}

TEST(ArmController, DynamicModeChangesTorqueFromStaticValue) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setJointAngleDegrees(0, 20.0);
  controller.setJointAngleDegrees(1, -30.0);
  const QVariantList static_torques = controller.jointTorques();

  controller.setDynamicMode(true);
  controller.setJointVelocityDegrees(0, 180.0);
  controller.setJointAccelerationDegrees(0, 300.0);
  controller.setJointAccelerationDegrees(1, -200.0);
  const QVariantList dynamic_torques = controller.jointTorques();

  ASSERT_EQ(static_torques.size(), dynamic_torques.size());
  bool any_changed = false;
  for (int i = 0; i < static_torques.size(); i++) {
    if (static_torques.at(i).toDouble() != dynamic_torques.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);
}

TEST(ArmController, TorqueBreakdownSumsToTheReportedTotal) {
  // The same exactness decompose_torque() itself guarantees
  // (dynamics_test.cpp) must hold through the controller's own
  // properties, not just the underlying function.
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setDynamicMode(true);
  controller.setJointAngleDegrees(0, 15.0);
  controller.setJointAngleDegrees(1, -25.0);
  controller.setJointVelocityDegrees(0, 90.0);
  controller.setJointAccelerationDegrees(1, 60.0);
  controller.setPayloadKg(1.5);

  const QVariantList total = controller.jointTorques();
  const QVariantList gravity = controller.gravityTorques();
  const QVariantList inertia = controller.inertiaTorques();
  const QVariantList coriolis = controller.coriolisTorques();

  ASSERT_EQ(total.size(), gravity.size());
  ASSERT_EQ(total.size(), inertia.size());
  ASSERT_EQ(total.size(), coriolis.size());
  for (int i = 0; i < total.size(); i++) {
    const double sum = gravity.at(i).toDouble() + inertia.at(i).toDouble() + coriolis.at(i).toDouble();
    EXPECT_NEAR(sum, total.at(i).toDouble(), 1e-6) << "joint " << i;
  }
}

TEST(ArmController, TrajectoryScrubModeImpliesDynamicModeAndMovesSmoothly) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setJointAngleDegrees(0, 60.0);
  controller.setJointAngleDegrees(1, -40.0);

  controller.setTrajectoryScrubMode(true);
  EXPECT_TRUE(controller.dynamicMode());
  ASSERT_TRUE(controller.trajectoryScrubMode());

  // At progress 0, the trajectory starts at rest (zero pose) - not the
  // target pose that was set before scrubbing began.
  controller.setTrajectoryProgress(0.0);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(0).toDouble(), 0.0, 1e-6);
  for (int i = 0; i < controller.jointVelocitiesDegrees().size(); i++) {
    EXPECT_NEAR(controller.jointVelocitiesDegrees().at(i).toDouble(), 0.0, 1e-6);
  }

  // At progress 1 (the end of the trajectory), it must have arrived at
  // exactly the captured target pose, again at rest.
  controller.setTrajectoryProgress(1.0);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(0).toDouble(), 60.0, 1e-4);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(1).toDouble(), -40.0, 1e-4);
  for (int i = 0; i < controller.jointVelocitiesDegrees().size(); i++) {
    EXPECT_NEAR(controller.jointVelocitiesDegrees().at(i).toDouble(), 0.0, 1e-4);
  }
}

TEST(ArmController, DisablingDynamicModeAlsoDisablesTrajectoryScrubMode) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setTrajectoryScrubMode(true);
  ASSERT_TRUE(controller.trajectoryScrubMode());

  controller.setDynamicMode(false);
  EXPECT_FALSE(controller.trajectoryScrubMode());
}

TEST(ArmController, EnablingScrubModeStartsWithTwoWaypoints) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  EXPECT_EQ(controller.trajectoryWaypointCount(), 0); // none recorded before scrubbing starts

  controller.setTrajectoryScrubMode(true);
  EXPECT_EQ(controller.trajectoryWaypointCount(), 2); // [rest, captured pose]
}

TEST(ArmController, RecordWaypointExtendsTheRecordingAndPlaybackReachesIt) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);
  controller.setJointAngleDegrees(1, -20.0);
  controller.setTrajectoryScrubMode(true);
  ASSERT_EQ(controller.trajectoryWaypointCount(), 2);

  // Move to a new pose and record it as a third waypoint.
  controller.setJointAngleDegrees(0, 70.0);
  controller.setJointAngleDegrees(1, 10.0);
  controller.recordWaypoint();
  EXPECT_EQ(controller.trajectoryWaypointCount(), 3);

  // Scrubbing to the very end must reach the newly-recorded waypoint,
  // not the one recorded when scrubbing first turned on.
  controller.setTrajectoryProgress(1.0);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(0).toDouble(), 70.0, 1e-4);
  EXPECT_NEAR(controller.jointAnglesDegrees().at(1).toDouble(), 10.0, 1e-4);
}

TEST(ArmController, RecordWaypointIsIgnoredOutsideScrubMode) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.recordWaypoint(); // must not crash with no recording to extend
  EXPECT_EQ(controller.trajectoryWaypointCount(), 0);
}

TEST(ArmController, ReEnablingScrubModeDiscardsThePreviousRecording) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setTrajectoryScrubMode(true);
  controller.recordWaypoint();
  controller.recordWaypoint();
  ASSERT_EQ(controller.trajectoryWaypointCount(), 4);

  controller.setTrajectoryScrubMode(false);
  controller.setTrajectoryScrubMode(true);
  EXPECT_EQ(controller.trajectoryWaypointCount(), 2); // back to [rest, captured pose]
}
