// Verifies SensorBridgeServer::handleMessage() - the JSON dispatch logic
// external sensor input (an IMU wand, a camera-based gripper controller)
// goes through - without needing a real WebSocket connection.
#include <string>

#include <gtest/gtest.h>
#include <QVariantList>
#include <QVector3D>

#include "arm_controller.hpp"
#include "sensor_bridge_server.hpp"

using robotarm::ArmController;
using robotarm::SensorBridgeServer;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string config_path() { return std::string(PROJECT_ROOT_DIR) + "/config/two_link_planar.json"; }
std::string surrogate_path() { return std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar.surrogate.json"; }

// Port is never actually bound in these tests - handleMessage() is called
// directly, bypassing the socket layer entirely.
constexpr quint16 kUnusedTestPort = 0;

} // namespace

TEST(SensorBridgeServer, JointAngleMessageSetsTheTargetJoint) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);

  bridge.handleMessage(R"({"type":"jointAngle","joint":0,"deg":30})");

  const QVariantList angles = controller.jointAnglesDegrees();
  ASSERT_EQ(angles.size(), 2);
  EXPECT_NEAR(angles[0].toDouble(), 30.0, 1e-6);
}

TEST(SensorBridgeServer, MoveTargetMessageSolvesInverseKinematicsToTheRequestedPoint) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);

  // config/two_link_planar.json: a=1.0 and a=0.5, total reach 1.5, default
  // pose fully extended along +X - a target straight out along +X is
  // exactly the solver's zero-gradient singularity direction from there
  // (see the project's own inverse-kinematics test history), so this picks
  // an off-axis point instead: distance sqrt(0.5^2+1^2) ~= 1.12, within
  // reach but not colinear with the starting pose.
  bridge.handleMessage(R"({"type":"moveTarget","x":0.5,"y":1.0})");

  const QVariantList positions = controller.jointPositions();
  ASSERT_FALSE(positions.isEmpty());
  const QVector3D tip = positions.last().value<QVector3D>();
  EXPECT_NEAR(tip.x(), 0.5f, 1e-2f);
  EXPECT_NEAR(tip.y(), 1.0f, 1e-2f);
}

TEST(SensorBridgeServer, GripperMessageTogglesGripperOpen) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  ASSERT_FALSE(controller.gripperOpen());

  bridge.handleMessage(R"({"type":"gripper","open":true})");
  EXPECT_TRUE(controller.gripperOpen());

  bridge.handleMessage(R"({"type":"gripper","open":false})");
  EXPECT_FALSE(controller.gripperOpen());
}

TEST(SensorBridgeServer, PingMessageChangesNothing) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  const QVariantList before = controller.jointAnglesDegrees();

  bridge.handleMessage(R"({"type":"ping"})");

  const QVariantList after = controller.jointAnglesDegrees();
  ASSERT_EQ(before.size(), after.size());
  for (int i = 0; i < before.size(); i++) EXPECT_NEAR(before[i].toDouble(), after[i].toDouble(), 1e-9);
}

TEST(SensorBridgeServer, MalformedJsonIsIgnoredWithoutCrashing) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  const QVariantList before = controller.jointAnglesDegrees();

  bridge.handleMessage(QStringLiteral("not json at all"));

  const QVariantList after = controller.jointAnglesDegrees();
  ASSERT_EQ(before.size(), after.size());
  for (int i = 0; i < before.size(); i++) EXPECT_NEAR(before[i].toDouble(), after[i].toDouble(), 1e-9);
}

TEST(SensorBridgeServer, UnknownMessageTypeIsIgnored) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  const QVariantList before = controller.jointAnglesDegrees();

  bridge.handleMessage(R"({"type":"doSomethingUnsupported","joint":0,"deg":45})");

  const QVariantList after = controller.jointAnglesDegrees();
  ASSERT_EQ(before.size(), after.size());
  for (int i = 0; i < before.size(); i++) EXPECT_NEAR(before[i].toDouble(), after[i].toDouble(), 1e-9);
}

TEST(SensorBridgeServer, JointAngleMessageMissingFieldsIsIgnored) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  const QVariantList before = controller.jointAnglesDegrees();

  bridge.handleMessage(R"({"type":"jointAngle","joint":0})"); // no "deg"

  const QVariantList after = controller.jointAnglesDegrees();
  ASSERT_EQ(before.size(), after.size());
  for (int i = 0; i < before.size(); i++) EXPECT_NEAR(before[i].toDouble(), after[i].toDouble(), 1e-9);
}

TEST(SensorBridgeServer, OutOfRangeJointIndexDoesNotCrashOrMutateOtherJoints) {
  ArmController controller(config_path(), surrogate_path());
  SensorBridgeServer bridge(&controller, kUnusedTestPort);
  const QVariantList before = controller.jointAnglesDegrees();

  bridge.handleMessage(R"({"type":"jointAngle","joint":99,"deg":10})");

  const QVariantList after = controller.jointAnglesDegrees();
  ASSERT_EQ(before.size(), after.size());
  for (int i = 0; i < before.size(); i++) EXPECT_NEAR(before[i].toDouble(), after[i].toDouble(), 1e-9);
}
