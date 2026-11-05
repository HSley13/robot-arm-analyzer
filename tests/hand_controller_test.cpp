// Verifies HandController: the shipped config loads with the right chain
// structure, direct joint edits clamp to the configured range and leave a
// fixed-placement joint alone, forward kinematics actually updates when a
// joint moves, and playGesture() drives a real, non-instant animation
// that settles at the target pose rather than teleporting there.
#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <QVariantList>
#include <QVector3D>

#include "hand_controller.hpp"

using robotarm::HandController;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string hand_config_path() { return std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"; }

} // namespace

TEST(HandController, LoadsTheShippedConfigWithExpectedChains) {
  HandController hand(hand_config_path());
  const QStringList names = hand.chainNames();

  EXPECT_EQ(names.size(), 6);
  for (const char *expected : {"wrist", "index", "middle", "ring", "pinky", "thumb"}) {
    EXPECT_TRUE(names.contains(QString(expected))) << "missing chain '" << expected << "'";
  }
}

TEST(HandController, RestsAtZeroOnConstruction) {
  HandController hand(hand_config_path());
  for (int i = 0; i < hand.jointCountForChain("index"); i++) {
    EXPECT_DOUBLE_EQ(hand.jointAngleDegrees("index", i), 0.0);
  }
}

TEST(HandController, SetJointAngleDegreesUpdatesTheAngleAndTheKinematics) {
  HandController hand(hand_config_path());
  // index joint 1 is the MCP (a real, movable joint - see
  // config/humanoid_hand.json): min_deg -15, max_deg 90.
  const QVariantList before = hand.jointPositionsForChain("index");

  hand.setJointAngleDegrees("index", 1, 45.0);

  EXPECT_NEAR(hand.jointAngleDegrees("index", 1), 45.0, 1e-9);
  const QVariantList after = hand.jointPositionsForChain("index");
  ASSERT_EQ(before.size(), after.size());
  EXPECT_NE(before.last().value<QVector3D>(), after.last().value<QVector3D>());
}

TEST(HandController, SetJointAngleDegreesClampsToTheConfiguredRange) {
  HandController hand(hand_config_path());
  hand.setJointAngleDegrees("index", 1, 999.0); // way past max_deg (90)
  EXPECT_NEAR(hand.jointAngleDegrees("index", 1), 90.0, 1e-9);

  hand.setJointAngleDegrees("index", 1, -999.0); // way past min_deg (-15)
  EXPECT_NEAR(hand.jointAngleDegrees("index", 1), -15.0, 1e-9);
}

TEST(HandController, SetJointAngleDegreesIgnoresAFixedPlacementJoint) {
  HandController hand(hand_config_path());
  // index joint 0 is the metacarpal placement joint: min_deg == max_deg == 0.
  ASSERT_DOUBLE_EQ(hand.jointMinDegrees("index", 0), hand.jointMaxDegrees("index", 0));

  hand.setJointAngleDegrees("index", 0, 45.0);
  EXPECT_DOUBLE_EQ(hand.jointAngleDegrees("index", 0), 0.0);
}

TEST(HandController, SetJointAngleDegreesOnUnknownChainOrIndexDoesNotCrash) {
  HandController hand(hand_config_path());
  hand.setJointAngleDegrees("not_a_real_chain", 0, 10.0);
  hand.setJointAngleDegrees("index", 999, 10.0);
  SUCCEED();
}

TEST(HandController, PlayGestureAnimatesRatherThanTeleporting) {
  HandController hand(hand_config_path());
  EXPECT_FALSE(hand.animating());

  // Expect a benign "Timers can only be used with threads started with
  // QThread" warning on stderr here - this plain-GTest binary has no
  // QCoreApplication/event loop, so the QTimer this starts can never
  // actually fire. That's fine: it's exactly what makes the assertion
  // below meaningful (see the comment there).
  hand.playGesture("closedFist");
  EXPECT_TRUE(hand.animating());
  EXPECT_EQ(hand.currentGesture(), QString("closedFist"));
  // playGesture() only arms the QTimer-driven animation; it must not
  // synchronously snap the pose to the target itself (that would be a
  // teleport, not an animation) - advanceAnimation() moves it, and that
  // only runs once the timer actually ticks, on a running event loop
  // this plain-GTest process never starts. So right after the call
  // returns, the pose must still be exactly at its pre-gesture value.
  EXPECT_NEAR(hand.jointAngleDegrees("index", 1), 0.0, 1e-9);
}

TEST(HandController, UnknownGestureNameIsIgnored) {
  HandController hand(hand_config_path());
  hand.playGesture("doTheRobot");
  EXPECT_FALSE(hand.animating());
  EXPECT_EQ(hand.currentGesture(), QString("restPose"));
}

TEST(HandController, LinkGeometryHasOneFewerEntryThanJointPositions) {
  HandController hand(hand_config_path());
  const int joint_count = hand.jointCountForChain("index");
  ASSERT_GT(joint_count, 1);

  EXPECT_EQ(hand.jointPositionsForChain("index").size(), joint_count);
  EXPECT_EQ(hand.linkMidpointsForChain("index").size(), joint_count - 1);
  EXPECT_EQ(hand.linkLengthsForChain("index").size(), joint_count - 1);
  EXPECT_EQ(hand.linkRotationsForChain("index").size(), joint_count - 1);
}

TEST(HandController, LinkLengthsMatchTheDistanceBetweenConsecutiveJointPositions) {
  HandController hand(hand_config_path());
  const QVariantList positions = hand.jointPositionsForChain("index");
  const QVariantList lengths = hand.linkLengthsForChain("index");
  ASSERT_EQ(lengths.size(), positions.size() - 1);

  for (int i = 0; i < lengths.size(); i++) {
    const QVector3D a = positions[i].value<QVector3D>();
    const QVector3D b = positions[i + 1].value<QVector3D>();
    EXPECT_NEAR(lengths[i].toDouble(), static_cast<double>((b - a).length()), 1e-6);
  }
}

TEST(HandController, LinkGeometryForUnknownChainIsEmptyNotACrash) {
  HandController hand(hand_config_path());
  EXPECT_TRUE(hand.linkMidpointsForChain("not_a_real_chain").isEmpty());
  EXPECT_TRUE(hand.linkLengthsForChain("not_a_real_chain").isEmpty());
  EXPECT_TRUE(hand.linkRotationsForChain("not_a_real_chain").isEmpty());
}

TEST(HandController, RootSegmentConnectsAFingerToTheWristsTip) {
  HandController hand(hand_config_path());
  const QVariantList wrist_positions = hand.jointPositionsForChain("wrist");
  const QVariantList finger_positions = hand.jointPositionsForChain("index");
  ASSERT_FALSE(wrist_positions.isEmpty());
  ASSERT_FALSE(finger_positions.isEmpty());

  const QVariantList mid = hand.rootSegmentMidpointForChain("index");
  const QVariantList len = hand.rootSegmentLengthForChain("index");
  const QVariantList rot = hand.rootSegmentRotationForChain("index");
  ASSERT_EQ(mid.size(), 1);
  ASSERT_EQ(len.size(), 1);
  ASSERT_EQ(rot.size(), 1);

  const QVector3D wrist_tip = wrist_positions.last().value<QVector3D>();
  const QVector3D finger_first = finger_positions.first().value<QVector3D>();
  const QVector3D expected_midpoint = (wrist_tip + finger_first) / 2.0f;
  EXPECT_NEAR(mid[0].value<QVector3D>().distanceToPoint(expected_midpoint), 0.0, 1e-6);
  EXPECT_NEAR(len[0].toDouble(), static_cast<double>((finger_first - wrist_tip).length()), 1e-6);
  EXPECT_GT(len[0].toDouble(), 0.0); // a finger's metacarpal offset is a real, nonzero length
}

TEST(HandController, RootSegmentIsAbsentForTheRootChainItself) {
  HandController hand(hand_config_path());
  EXPECT_TRUE(hand.rootSegmentMidpointForChain("wrist").isEmpty());
  EXPECT_TRUE(hand.rootSegmentLengthForChain("wrist").isEmpty());
  EXPECT_TRUE(hand.rootSegmentRotationForChain("wrist").isEmpty());
}

TEST(HandController, RootSegmentIsAbsentForAnUnknownChain) {
  HandController hand(hand_config_path());
  EXPECT_TRUE(hand.rootSegmentMidpointForChain("not_a_real_chain").isEmpty());
}

TEST(HandController, RevisionIncrementsOnADirectJointEdit) {
  HandController hand(hand_config_path());
  const int before = hand.property("revision").toInt();
  hand.setJointAngleDegrees("index", 1, 30.0);
  const int after = hand.property("revision").toInt();
  EXPECT_GT(after, before);
}
