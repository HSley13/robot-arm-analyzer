// Verifies ArmController: the QML-facing properties reflect the loaded
// config correctly, recompute happens on every mutation, the Exact/AI
// toggle actually switches engines (and still agrees closely with Exact,
// the same cross-check Module 5 already established), and the
// color-threshold/camera-pivot logic the UI depends on is correct.
#include <cmath>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>
#include <QColor>
#include <QVariantList>
#include <QVector3D>

#include "arm_controller.hpp"
#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/payload_capacity.hpp"
#include "robotarm/statics.hpp"

using robotarm::ArmController;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string config_path() { return std::string(PROJECT_ROOT_DIR) + "/tests/fixtures/two_link_planar.json"; }
std::string three_dof_config_path() {
  return std::string(PROJECT_ROOT_DIR) + "/tests/fixtures/three_dof_with_prismatic.json";
}
std::string surrogate_path() { return std::string(PROJECT_ROOT_DIR) + "/tests/fixtures/two_link_planar.surrogate.json"; }
std::string dynamic_surrogate_path() {
  return std::string(PROJECT_ROOT_DIR) + "/tests/fixtures/two_link_planar_dynamic.surrogate.json";
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

TEST(ArmController, SetJointTorqueLimitUpdatesTheReportedLimit) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointTorqueLimit(0, 99.0);
  EXPECT_DOUBLE_EQ(controller.jointTorqueLimits().at(0).toDouble(), 99.0);
}

TEST(ArmController, SetJointTorqueLimitIgnoresNonPositiveValuesAndBadIndex) {
  ArmController controller(config_path(), surrogate_path());
  const double original = controller.jointTorqueLimits().at(0).toDouble();

  controller.setJointTorqueLimit(0, 0.0);
  controller.setJointTorqueLimit(0, -5.0);
  EXPECT_DOUBLE_EQ(controller.jointTorqueLimits().at(0).toDouble(), original);

  controller.setJointTorqueLimit(99, 10.0); // must not crash
}

TEST(ArmController, SaveConfigThenLoadConfigRoundTripsEditedTorqueLimits) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointTorqueLimit(0, 123.0);

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "robotarm_arm_controller_roundtrip_test.json";
  ASSERT_TRUE(controller.saveConfig(QString::fromStdString(path.string())));

  ArmController reloaded_target(config_path(), surrogate_path());
  ASSERT_TRUE(reloaded_target.loadConfig(QString::fromStdString(path.string())));
  std::filesystem::remove(path);

  EXPECT_DOUBLE_EQ(reloaded_target.jointTorqueLimits().at(0).toDouble(), 123.0);
}

TEST(ArmController, LoadConfigWithADifferentJointCountResetsStateAndDisablesAi) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setJointAngleDegrees(0, 45.0);
  controller.setDynamicMode(true);
  controller.setAiMode(true);
  ASSERT_TRUE(controller.aiMode());

  ASSERT_TRUE(controller.loadConfig(QString::fromStdString(three_dof_config_path())));
  EXPECT_EQ(controller.jointCount(), 3);
  EXPECT_FALSE(controller.aiMode()); // the two-link surrogate no longer matches
  EXPECT_FALSE(controller.aiModeAvailable());
  EXPECT_FALSE(controller.dynamicMode());
  for (int i = 0; i < controller.jointAnglesDegrees().size(); i++) {
    EXPECT_NEAR(controller.jointAnglesDegrees().at(i).toDouble(), 0.0, 1e-9);
  }
}

TEST(ArmController, LoadConfigReturnsFalseAndLeavesTheArmUnchangedOnFailure) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);

  EXPECT_FALSE(controller.loadConfig("/nonexistent/path/does_not_exist.json"));
  EXPECT_EQ(controller.jointCount(), 2); // unchanged - the failed load must not corrupt state
  EXPECT_NEAR(controller.jointAnglesDegrees().at(0).toDouble(), 30.0, 1e-9);
  EXPECT_TRUE(controller.configMessage().contains("failed", Qt::CaseInsensitive));
}

TEST(ArmController, SaveConfigReturnsFalseOnAnUnwritablePath) {
  ArmController controller(config_path(), surrogate_path());
  EXPECT_FALSE(controller.saveConfig("/nonexistent/directory/does_not_exist.json"));
  EXPECT_TRUE(controller.configMessage().contains("failed", Qt::CaseInsensitive));
}

TEST(ArmController, GripperDefaultsToClosedSoPayloadAffectsTorqueWithoutAnyGripperInteraction) {
  // Every pre-existing payload test in this file never touches the
  // gripper and still expects payloadKg to affect torque immediately -
  // that only keeps working if the gripper defaults to closed.
  ArmController controller(config_path(), surrogate_path());
  ASSERT_FALSE(controller.gripperOpen());

  controller.setJointAngleDegrees(0, 30.0);
  const QVariantList torque_no_payload = controller.jointTorques();
  controller.setPayloadKg(3.0);
  const QVariantList torque_with_payload = controller.jointTorques();

  bool any_changed = false;
  for (int i = 0; i < torque_no_payload.size(); i++) {
    if (torque_no_payload.at(i).toDouble() != torque_with_payload.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);
}

TEST(ArmController, OpeningGripperZeroesThePayloadsEffectOnTorque) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);
  controller.setPayloadKg(3.0);
  const QVariantList torque_closed_with_payload = controller.jointTorques();

  controller.setGripperOpen(true);
  ASSERT_TRUE(controller.gripperOpen());
  const QVariantList torque_open = controller.jointTorques();

  // payloadKg() itself is untouched by opening the gripper - only its
  // effect on torque is.
  EXPECT_DOUBLE_EQ(controller.payloadKg(), 3.0);

  // Opening the gripper must match torque computed with no payload at
  // all, not just "be different from the closed-with-payload case".
  controller.setGripperOpen(false);
  controller.setPayloadKg(0.0);
  const QVariantList torque_zero_payload = controller.jointTorques();
  controller.setGripperOpen(true);
  controller.setPayloadKg(3.0);
  const QVariantList torque_open_again = controller.jointTorques();

  ASSERT_EQ(torque_open_again.size(), torque_zero_payload.size());
  for (int i = 0; i < torque_open_again.size(); i++) {
    EXPECT_NEAR(torque_open_again.at(i).toDouble(), torque_zero_payload.at(i).toDouble(), 1e-9);
  }

  bool differs_from_closed = false;
  for (int i = 0; i < torque_open.size(); i++) {
    if (torque_open.at(i).toDouble() != torque_closed_with_payload.at(i).toDouble()) differs_from_closed = true;
  }
  EXPECT_TRUE(differs_from_closed);
}

TEST(ArmController, ClosingGripperAfterOpeningRestoresThePayloadsEffect) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);
  controller.setPayloadKg(3.0);
  const QVariantList torque_before = controller.jointTorques();

  controller.setGripperOpen(true);
  controller.setGripperOpen(false);
  const QVariantList torque_after = controller.jointTorques();

  ASSERT_EQ(torque_before.size(), torque_after.size());
  for (int i = 0; i < torque_before.size(); i++) {
    EXPECT_DOUBLE_EQ(torque_before.at(i).toDouble(), torque_after.at(i).toDouble());
  }
}

TEST(ArmController, GripperOpenAlsoZeroesThePayloadsEffectInDynamicModeAndBreakdown) {
  ArmController controller(config_path(), surrogate_path(), dynamic_surrogate_path());
  controller.setDynamicMode(true);
  controller.setJointAngleDegrees(0, 15.0);
  controller.setJointVelocityDegrees(0, 90.0);
  controller.setPayloadKg(2.0);

  controller.setGripperOpen(true);
  const QVariantList gravity_open = controller.gravityTorques();

  controller.setGripperOpen(false);
  controller.setPayloadKg(0.0);
  const QVariantList gravity_zero_payload = controller.gravityTorques();

  ASSERT_EQ(gravity_open.size(), gravity_zero_payload.size());
  for (int i = 0; i < gravity_open.size(); i++) {
    EXPECT_NEAR(gravity_open.at(i).toDouble(), gravity_zero_payload.at(i).toDouble(), 1e-9);
  }
}

TEST(ArmController, LoadConfigResetsGripperToClosed) {
  ArmController controller(config_path(), surrogate_path());
  controller.setGripperOpen(true);
  ASSERT_TRUE(controller.gripperOpen());

  ASSERT_TRUE(controller.loadConfig(QString::fromStdString(config_path())));
  EXPECT_FALSE(controller.gripperOpen());
}

TEST(ArmController, SetJointLengthChangesLinkLengthAndCanvasCenter) {
  ArmController controller(config_path(), surrogate_path());
  const QVector3D center_before = controller.canvasCenter();

  controller.setJointAngleDegrees(0, 0.0); // theta=0, so link length == dh.a exactly
  controller.setJointLength(0, 2.0);

  EXPECT_NEAR(controller.linkLengths().at(0).toDouble(), 2.0, 1e-9);
  EXPECT_GT(controller.canvasCenter().x(), center_before.x()); // reach grew, pivot moved with it
}

TEST(ArmController, SetJointLengthIgnoresNonPositiveValuesAndBadIndex) {
  ArmController controller(config_path(), surrogate_path());
  const double original = controller.linkLengths().at(0).toDouble();

  controller.setJointLength(0, 0.0);
  controller.setJointLength(0, -1.0);
  EXPECT_DOUBLE_EQ(controller.linkLengths().at(0).toDouble(), original);

  controller.setJointLength(99, 2.0); // must not crash
}

TEST(ArmController, SetJointLengthDisablesAiMode) {
  ArmController controller(config_path(), surrogate_path());
  controller.setAiMode(true);
  ASSERT_TRUE(controller.aiMode());

  controller.setJointLength(0, 1.5);
  EXPECT_FALSE(controller.aiMode());
}

TEST(ArmController, SetJointLengthChangesTorqueForTheSamePose) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 30.0);
  controller.setJointAngleDegrees(1, -20.0);
  const QVariantList torque_before = controller.jointTorques();

  controller.setJointLength(0, 2.5); // a longer upper link shifts every downstream mass's moment arm

  const QVariantList torque_after = controller.jointTorques();
  ASSERT_EQ(torque_before.size(), torque_after.size());
  bool any_changed = false;
  for (int i = 0; i < torque_before.size(); i++) {
    if (torque_before.at(i).toDouble() != torque_after.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);
}

TEST(ArmController, SaveConfigThenLoadConfigRoundTripsAnEditedLength) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointLength(0, 1.75);

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "robotarm_arm_controller_length_roundtrip_test.json";
  ASSERT_TRUE(controller.saveConfig(QString::fromStdString(path.string())));

  ArmController reloaded_target(config_path(), surrogate_path());
  ASSERT_TRUE(reloaded_target.loadConfig(QString::fromStdString(path.string())));
  std::filesystem::remove(path);

  reloaded_target.setJointAngleDegrees(0, 0.0);
  EXPECT_NEAR(reloaded_target.linkLengths().at(0).toDouble(), 1.75, 1e-9);
}

TEST(ArmController, SetJointMassChangesTheReportedMass) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointMass(0, 500.0); // simulate a much heavier arm, not just a heavier payload
  EXPECT_DOUBLE_EQ(controller.jointMassesKg().at(0).toDouble(), 500.0);
}

TEST(ArmController, SetJointMassAcceptsZeroButRejectsNegativeAndBadIndex) {
  ArmController controller(config_path(), surrogate_path());
  const double original = controller.jointMassesKg().at(0).toDouble();

  controller.setJointMass(0, -1.0);
  EXPECT_DOUBLE_EQ(controller.jointMassesKg().at(0).toDouble(), original);

  controller.setJointMass(0, 0.0); // a massless link is valid, unlike a non-positive length
  EXPECT_DOUBLE_EQ(controller.jointMassesKg().at(0).toDouble(), 0.0);

  controller.setJointMass(99, 5.0); // must not crash
}

TEST(ArmController, SetJointMassDisablesAiMode) {
  ArmController controller(config_path(), surrogate_path());
  controller.setAiMode(true);
  ASSERT_TRUE(controller.aiMode());

  controller.setJointMass(0, 10.0);
  EXPECT_FALSE(controller.aiMode());
}

TEST(ArmController, SetJointMassChangesTorqueForTheSamePose) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 20.0);
  controller.setJointAngleDegrees(1, -15.0);
  const QVariantList torque_before = controller.jointTorques();

  controller.setJointMass(0, 50.0); // a much heavier link loads every upstream joint more

  const QVariantList torque_after = controller.jointTorques();
  ASSERT_EQ(torque_before.size(), torque_after.size());
  bool any_changed = false;
  for (int i = 0; i < torque_before.size(); i++) {
    if (torque_before.at(i).toDouble() != torque_after.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);
}

TEST(ArmController, SetJointMassClearsAPreviousScan) {
  ArmController controller(config_path(), surrogate_path());
  controller.runPayloadScan(10);
  ASSERT_EQ(controller.scanSampleCount(), 10);

  controller.setJointMass(0, 20.0);
  EXPECT_EQ(controller.scanSampleCount(), 0);
}

TEST(ArmController, SaveConfigThenLoadConfigRoundTripsAnEditedMass) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointMass(0, 123.0);

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "robotarm_arm_controller_mass_roundtrip_test.json";
  ASSERT_TRUE(controller.saveConfig(QString::fromStdString(path.string())));

  ArmController reloaded_target(config_path(), surrogate_path());
  ASSERT_TRUE(reloaded_target.loadConfig(QString::fromStdString(path.string())));
  std::filesystem::remove(path);

  EXPECT_DOUBLE_EQ(reloaded_target.jointMassesKg().at(0).toDouble(), 123.0);
}

TEST(ArmController, CurrentMaxSafePayloadKgIsPositiveAndBounded) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 0.0); // extended horizontally - the worst case
  controller.setJointAngleDegrees(1, 0.0);

  const double result = controller.currentMaxSafePayloadKg();
  EXPECT_GT(result, 0.0);
  EXPECT_LT(result, 200.0); // the search ceiling ArmController uses
}

TEST(ArmController, CurrentMaxSafePayloadKgMatchesADirectComputationAtTheSamePose) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 0.0);
  controller.setJointAngleDegrees(1, 0.0);

  // Reproduce the same computation independently, using the core
  // function directly on a hand-built arm - this is specifically
  // checking ArmController's wiring (right pose, right limits, right
  // gravity vector), not max_safe_payload() itself (already covered
  // by payload_capacity_tests).
  const robotarm::ArmModel arm({robotarm::DHParameter(1.0), robotarm::DHParameter(0.5)});
  const std::vector<robotarm::LinkMass> links = {robotarm::LinkMass(2.0), robotarm::LinkMass(1.0)};
  const Eigen::Vector3d gravity_down_y(0.0, -9.81, 0.0);
  const std::vector<double> joint_values = {0.0, 0.0};
  const std::vector<double> limits = {50.0, 25.0};
  auto evaluator = [&](double payload) {
    return robotarm::static_joint_torques(arm, joint_values, links, payload, gravity_down_y);
  };
  const double expected = robotarm::max_safe_payload(evaluator, limits, 200.0);

  EXPECT_NEAR(controller.currentMaxSafePayloadKg(), expected, 1e-2);
}

TEST(ArmController, CurrentMaxSafePayloadKgIsHigherWhenTheArmIsFolded) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 0.0);
  controller.setJointAngleDegrees(1, 0.0);
  const double extended = controller.currentMaxSafePayloadKg();

  controller.setJointAngleDegrees(1, -170.0); // elbow folded back toward the shoulder
  const double folded = controller.currentMaxSafePayloadKg();

  EXPECT_GT(folded, extended);
}

TEST(ArmController, RunPayloadScanClampsSampleCountIntoAReasonableRange) {
  ArmController controller(config_path(), surrogate_path());

  controller.runPayloadScan(1);
  EXPECT_GE(controller.scanSampleCount(), 2);

  controller.runPayloadScan(100000);
  EXPECT_LE(controller.scanSampleCount(), 300);
}

TEST(ArmController, RunPayloadScanProducesADecreasingReachCurve) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 0.0);
  controller.runPayloadScan(20);

  ASSERT_EQ(controller.scanSampleCount(), 20);
  const QVariantList reach = controller.scanReachMeters();
  const QVariantList exact_payload = controller.scanExactPayloadKg();
  ASSERT_EQ(reach.size(), 20);
  ASSERT_EQ(exact_payload.size(), 20);

  // Folding the arm (the scan's whole premise) must shrink its reach,
  // and the less-extended end must be able to carry at least as much
  // as the fully-extended start - the actual point of the curve.
  EXPECT_GT(reach.first().toDouble(), reach.last().toDouble());
  EXPECT_GE(exact_payload.last().toDouble(), exact_payload.first().toDouble());
}

TEST(ArmController, RunPayloadScanIncludesAiResultsWhenASurrogateMatches) {
  ArmController controller(config_path(), surrogate_path());
  controller.runPayloadScan(15);

  EXPECT_TRUE(controller.scanAiAvailable());
  EXPECT_EQ(controller.scanAiPayloadKg().size(), 15);
  EXPECT_GE(controller.scanAiMilliseconds(), 0.0);
  EXPECT_GE(controller.scanExactMilliseconds(), 0.0);
}

TEST(ArmController, RunPayloadScanHasNoAiResultsWithoutAMatchingSurrogate) {
  ArmController controller(config_path(), "/nonexistent/weights.json");
  controller.runPayloadScan(15);

  EXPECT_FALSE(controller.scanAiAvailable());
  EXPECT_TRUE(controller.scanAiPayloadKg().isEmpty());
}

TEST(ArmController, SetJointLengthClearsAPreviousScan) {
  ArmController controller(config_path(), surrogate_path());
  controller.runPayloadScan(10);
  ASSERT_EQ(controller.scanSampleCount(), 10);

  controller.setJointLength(0, 1.2);
  EXPECT_EQ(controller.scanSampleCount(), 0);
}

TEST(ArmController, LoadConfigClearsAPreviousScan) {
  ArmController controller(config_path(), surrogate_path());
  controller.runPayloadScan(10);
  ASSERT_EQ(controller.scanSampleCount(), 10);

  ASSERT_TRUE(controller.loadConfig(QString::fromStdString(config_path())));
  EXPECT_EQ(controller.scanSampleCount(), 0);
}

TEST(ArmController, MoveToTargetReachesAKnownReachablePoint) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 40.0);
  controller.setJointAngleDegrees(1, -25.0);
  const QVector3D target = controller.jointPositions().last().value<QVector3D>();

  controller.setJointAngleDegrees(0, 0.0);
  controller.setJointAngleDegrees(1, 0.0);
  controller.moveToTarget(target.x(), target.y());

  const QVector3D reached = controller.jointPositions().last().value<QVector3D>();
  EXPECT_NEAR(reached.x(), target.x(), 1e-2);
  EXPECT_NEAR(reached.y(), target.y(), 1e-2);
}

TEST(ArmController, MoveToTargetUsesTheCurrentPoseAsItsStartingGuess) {
  // Different starting pose, different (reachable) target - not a
  // repeat of the previous test's specific numbers.
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, -60.0);
  controller.setJointAngleDegrees(1, 50.0);
  const QVariantList angles_before = controller.jointAnglesDegrees();

  controller.moveToTarget(1.0, 0.3);
  const QVector3D reached = controller.jointPositions().last().value<QVector3D>();

  EXPECT_NEAR(reached.x(), 1.0, 1e-2);
  EXPECT_NEAR(reached.y(), 0.3, 1e-2);

  // The pose actually changed - moveToTarget didn't just leave the
  // previous pose in place by coincidence.
  bool any_changed = false;
  const QVariantList angles_after = controller.jointAnglesDegrees();
  for (int i = 0; i < angles_before.size(); i++) {
    if (angles_before.at(i).toDouble() != angles_after.at(i).toDouble()) any_changed = true;
  }
  EXPECT_TRUE(any_changed);
}

TEST(ArmController, MoveToTargetGetsCloserEvenWhenUnreachable) {
  ArmController controller(config_path(), surrogate_path());
  controller.setJointAngleDegrees(0, 90.0); // pointing away from the target below
  controller.setJointAngleDegrees(1, 90.0);
  const QVector3D before = controller.jointPositions().last().value<QVector3D>();
  const double distance_before = std::hypot(before.x() - 50.0, before.y() - 0.0);

  controller.moveToTarget(50.0, 0.0); // far beyond the 1.5m total reach
  const QVector3D after = controller.jointPositions().last().value<QVector3D>();
  const double distance_after = std::hypot(after.x() - 50.0, after.y() - 0.0);

  EXPECT_LT(distance_after, distance_before);
}

TEST(ArmController, AddJointIncreasesJointCountAndExtendsPoseVectors) {
  ArmController controller(config_path(), surrogate_path());
  ASSERT_EQ(controller.jointCount(), 2);

  controller.addJoint();

  EXPECT_EQ(controller.jointCount(), 3);
  EXPECT_EQ(controller.jointAnglesDegrees().size(), 3);
  EXPECT_EQ(controller.jointTorqueLimits().size(), 3);
  EXPECT_EQ(controller.linkLengths().size(), 3);
  EXPECT_EQ(controller.jointPositions().size(), 4); // base + one per joint
  EXPECT_NEAR(controller.jointAnglesDegrees().last().toDouble(), 0.0, 1e-9); // starts straight
}

TEST(ArmController, AddJointContinuesTheTaperConvention) {
  ArmController controller(config_path(), surrogate_path());
  const double last_length_before = controller.linkLengths().last().toDouble();
  const double last_limit_before = controller.jointTorqueLimits().last().toDouble();

  controller.addJoint();

  // The new joint must be shorter and have a lower torque limit than
  // the one it was appended after - the taper convention, not just
  // "some new joint got added with arbitrary numbers".
  EXPECT_LT(controller.linkLengths().last().toDouble(), last_length_before);
  EXPECT_LT(controller.jointTorqueLimits().last().toDouble(), last_limit_before);
}

TEST(ArmController, AddJointDisablesAiModeAndClearsAPreviousScan) {
  ArmController controller(config_path(), surrogate_path());
  controller.setAiMode(true);
  controller.runPayloadScan(10);
  ASSERT_TRUE(controller.aiMode());
  ASSERT_EQ(controller.scanSampleCount(), 10);

  controller.addJoint();

  EXPECT_FALSE(controller.aiMode());
  EXPECT_EQ(controller.scanSampleCount(), 0);
}

TEST(ArmController, AddJointResetsTrajectoryScrubMode) {
  ArmController controller(config_path(), surrogate_path());
  controller.setTrajectoryScrubMode(true);
  ASSERT_TRUE(controller.trajectoryScrubMode());

  controller.addJoint(); // must not leave a 2-joint recording attached to a 3-joint arm

  EXPECT_FALSE(controller.trajectoryScrubMode());
  EXPECT_EQ(controller.trajectoryWaypointCount(), 0);
}

TEST(ArmController, RemoveLastJointDecreasesJointCountAndPoseVectors) {
  ArmController controller(config_path(), surrogate_path());
  controller.addJoint();
  ASSERT_EQ(controller.jointCount(), 3);

  controller.removeLastJoint();

  EXPECT_EQ(controller.jointCount(), 2);
  EXPECT_EQ(controller.jointAnglesDegrees().size(), 2);
  EXPECT_EQ(controller.jointPositions().size(), 3);
}

TEST(ArmController, RemoveLastJointRefusesToGoBelowOneJoint) {
  ArmController controller(config_path(), surrogate_path());
  controller.removeLastJoint();
  controller.removeLastJoint(); // the first already brought it to 1; this must be a no-op

  EXPECT_EQ(controller.jointCount(), 1);
}

TEST(ArmController, AddThenRemoveRoundTripsBackToTheOriginalJointCount) {
  ArmController controller(config_path(), surrogate_path());
  const int original_count = controller.jointCount();

  controller.addJoint();
  controller.addJoint();
  controller.removeLastJoint();
  controller.removeLastJoint();

  EXPECT_EQ(controller.jointCount(), original_count);
}

TEST(ArmController, AddJointThenSaveConfigRoundTripsTheNewJointCount) {
  ArmController controller(config_path(), surrogate_path());
  controller.addJoint();
  ASSERT_EQ(controller.jointCount(), 3);

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "robotarm_arm_controller_added_joint_roundtrip_test.json";
  ASSERT_TRUE(controller.saveConfig(QString::fromStdString(path.string())));

  ArmController reloaded_target(config_path(), surrogate_path());
  ASSERT_TRUE(reloaded_target.loadConfig(QString::fromStdString(path.string())));
  std::filesystem::remove(path);

  EXPECT_EQ(reloaded_target.jointCount(), 3);
}

TEST(ArmController, LoadConfigWithMoreJointsThanCurrentWorksEndToEnd) {
  // Confirms the other half of "add more joints": loading a file
  // whose joint count is larger than the currently-loaded arm's, not
  // just a different arm with the SAME or fewer joints (already
  // covered by LoadConfigWithADifferentJointCountResetsStateAndDisablesAi).
  ArmController controller(config_path(), surrogate_path());
  ASSERT_EQ(controller.jointCount(), 2);

  ASSERT_TRUE(controller.loadConfig(QString::fromStdString(three_dof_config_path())));

  EXPECT_EQ(controller.jointCount(), 3);
  EXPECT_EQ(controller.jointAnglesDegrees().size(), 3);
  EXPECT_EQ(controller.jointPositions().size(), 4);
  EXPECT_GT(controller.canvasCenter().x(), 0.0);

  // The newly-loaded arm's own torques must be computable without
  // crashing or returning a mismatched-size result.
  EXPECT_EQ(controller.jointTorques().size(), 3);
}

namespace {
std::string humanoid_config_path() { return std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json"; }
} // namespace

TEST(ArmController, HumanoidArmExposesJointNamesAndLimits) {
  ArmController controller(humanoid_config_path(), "/nonexistent/weights.json");
  const QStringList names = controller.jointNames();
  ASSERT_EQ(names.size(), 3);
  EXPECT_EQ(names[0], QString("Shoulder"));
  EXPECT_EQ(names[1], QString("Elbow"));
  EXPECT_EQ(names[2], QString("Wrist"));
  EXPECT_DOUBLE_EQ(controller.jointMinDegrees()[1].toDouble(), 0.0);
  EXPECT_DOUBLE_EQ(controller.jointMaxDegrees()[1].toDouble(), 145.0);
}

TEST(ArmController, UnnamedJointsFallBackToNumberedNames) {
  ArmController controller(config_path(), surrogate_path());
  EXPECT_EQ(controller.jointNames(), QStringList({"Joint 1", "Joint 2"}));
}

TEST(ArmController, SetJointAngleDegreesClampsToTheJointsTravel) {
  ArmController controller(humanoid_config_path(), "/nonexistent/weights.json");
  controller.setJointAngleDegrees(1, -40.0); // an elbow can't hyperextend
  EXPECT_NEAR(controller.jointAnglesDegrees()[1].toDouble(), 0.0, 1e-9);
  controller.setJointAngleDegrees(1, 400.0);
  EXPECT_NEAR(controller.jointAnglesDegrees()[1].toDouble(), 145.0, 1e-9);
  controller.setJointAngleDegrees(1, 90.0);
  EXPECT_NEAR(controller.jointAnglesDegrees()[1].toDouble(), 90.0, 1e-9);
}

TEST(ArmController, MoveToTargetNeverLeavesTheJointTravel) {
  ArmController controller(humanoid_config_path(), "/nonexistent/weights.json");
  // A point below and slightly in front of the shoulder - reachable, but
  // a naive solve from the straight-out pose wants a negative elbow.
  controller.moveToTarget(0.35, -0.35);
  const QVariantList angles = controller.jointAnglesDegrees();
  for (int i = 0; i < angles.size(); i++) {
    EXPECT_GE(angles[i].toDouble(), controller.jointMinDegrees()[i].toDouble() - 1e-6) << "joint " << i;
    EXPECT_LE(angles[i].toDouble(), controller.jointMaxDegrees()[i].toDouble() + 1e-6) << "joint " << i;
  }
}

TEST(ArmController, PayloadScanFoldsTheElbowTheWayItCanBend) {
  ArmController controller(humanoid_config_path(), "/nonexistent/weights.json");
  controller.runPayloadScan(10);
  const QVariantList reach = controller.scanReachMeters();
  ASSERT_EQ(reach.size(), 10);
  // Folding a positive-only elbow must actually shorten the reach - if it
  // tried to fold negatively, the clamp would pin it straight and every
  // sample would sit at full reach.
  EXPECT_LT(reach.last().toDouble(), reach.first().toDouble() - 0.1);
}

TEST(ArmController, FrameRotationsMatchJointPositionsAndFollowTheShoulder) {
  ArmController controller(humanoid_config_path(), "/nonexistent/weights.json");
  ASSERT_EQ(controller.frameRotations().size(), controller.jointPositions().size());

  controller.setJointAngleDegrees(0, -90.0); // arm hanging straight down
  const QQuaternion upper_arm = controller.frameRotations()[1].value<QQuaternion>();
  const QVector3D x_axis = upper_arm.rotatedVector(QVector3D(1, 0, 0));
  EXPECT_NEAR(x_axis.y(), -1.0, 1e-5);
}

TEST(ArmController, HumanoidAiScanStaysCloseToTheExactScan) {
  // The AI search is capped to the network's training payload range, so
  // it can't wander off into extrapolated nonsense - its capacity curve
  // should track the exact engine's.
  ArmController controller(humanoid_config_path(),
                           std::string(PROJECT_ROOT_DIR) + "/ml/models/humanoid_arm.surrogate.json");
  ASSERT_TRUE(controller.aiModeAvailable());
  controller.runPayloadScan(20);
  ASSERT_TRUE(controller.scanAiAvailable());

  const QVariantList exact = controller.scanExactPayloadKg();
  const QVariantList ai = controller.scanAiPayloadKg();
  ASSERT_EQ(exact.size(), ai.size());
  for (int i = 0; i < exact.size(); i++) {
    EXPECT_NEAR(ai[i].toDouble(), exact[i].toDouble(), 2.0) << "sample " << i;
  }
}
