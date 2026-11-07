// Verifies HandGestures: restPose() is exactly zero everywhere, open/
// closed drive every real joint to its own min/max degrees while leaving
// a fixed-placement joint and the root chain untouched, and wave()
// swings only the root chain's first joint - including the degenerate
// case where the root chain has no joints at all.
#include <cmath>

#include <gtest/gtest.h>

#include "robotarm/hand_config.hpp"
#include "robotarm/hand_gestures.hpp"
#include "robotarm/units.hpp"

using robotarm::HandChainConfig;
using robotarm::HandConfig;
using robotarm::HandGestures;
using robotarm::HandJointConfig;
using robotarm::to_radians;

namespace {

HandJointConfig joint(double min_deg, double max_deg) {
  HandJointConfig j;
  j.min_degrees = min_deg;
  j.max_degrees = max_deg;
  return j;
}

// A minimal but representative hand: a wrist (root) with one real joint,
// and one finger chain with a fixed-placement joint followed by a real
// flexion joint.
HandConfig small_hand() {
  HandConfig config;
  config.name = "Test Hand";

  HandChainConfig wrist;
  wrist.name = "wrist";
  wrist.joints = {joint(-80.0, 80.0)};
  config.chains.push_back(wrist);

  HandChainConfig finger;
  finger.name = "index";
  finger.parent_chain = "wrist";
  finger.joints = {joint(0.0, 0.0), joint(-15.0, 90.0)}; // metacarpal placement, then MCP
  config.chains.push_back(finger);

  return config;
}

} // namespace

TEST(HandGestures, RestPoseIsExactlyZeroEverywhere) {
  const HandGestures gestures(small_hand());
  const auto pose = gestures.restPose();

  ASSERT_EQ(pose.at("wrist").size(), 1u);
  EXPECT_DOUBLE_EQ(pose.at("wrist")[0], 0.0);
  ASSERT_EQ(pose.at("index").size(), 2u);
  EXPECT_DOUBLE_EQ(pose.at("index")[0], 0.0);
  EXPECT_DOUBLE_EQ(pose.at("index")[1], 0.0);
}

TEST(HandGestures, OpenPalmDrivesTheRealFingerJointToItsMinAndLeavesTheRootAtZero) {
  const HandGestures gestures(small_hand());
  const auto pose = gestures.openPalm();

  EXPECT_DOUBLE_EQ(pose.at("wrist")[0], 0.0); // root chain excluded
  EXPECT_DOUBLE_EQ(pose.at("index")[0], 0.0); // fixed placement joint never moves
  EXPECT_NEAR(pose.at("index")[1], to_radians(-15.0), 1e-12);
}

TEST(HandGestures, ClosedFistDrivesTheRealFingerJointToItsMaxAndLeavesTheRootAtZero) {
  const HandGestures gestures(small_hand());
  const auto pose = gestures.closedFist();

  EXPECT_DOUBLE_EQ(pose.at("wrist")[0], 0.0);
  EXPECT_DOUBLE_EQ(pose.at("index")[0], 0.0);
  EXPECT_NEAR(pose.at("index")[1], to_radians(90.0), 1e-12);
}

TEST(HandGestures, WaveSwingsOnlyTheRootChainsFirstJoint) {
  const HandGestures gestures(small_hand());
  const auto waypoints = gestures.wave();

  ASSERT_GE(waypoints.size(), 2u);
  bool saw_a_nonzero_wrist_angle = false;
  for (const auto &pose : waypoints) {
    // The finger must never move during a wave.
    EXPECT_DOUBLE_EQ(pose.at("index")[0], 0.0);
    EXPECT_DOUBLE_EQ(pose.at("index")[1], 0.0);
    if (std::abs(pose.at("wrist")[0]) > 1e-9) saw_a_nonzero_wrist_angle = true;
  }
  EXPECT_TRUE(saw_a_nonzero_wrist_angle);

  // And it must actually be back-and-forth, not monotonic - at least two
  // waypoints should disagree with each other on the wrist angle.
  bool saw_a_difference = false;
  for (size_t i = 1; i < waypoints.size(); i++) {
    if (std::abs(waypoints[i].at("wrist")[0] - waypoints[i - 1].at("wrist")[0]) > 1e-9) saw_a_difference = true;
  }
  EXPECT_TRUE(saw_a_difference);
}

TEST(HandGestures, WaveWithNoRootJointsDoesNotCrash) {
  HandConfig config;
  HandChainConfig wrist;
  wrist.name = "wrist"; // root chain, zero joints - nothing to wave
  config.chains.push_back(wrist);

  const HandGestures gestures(config);
  const auto waypoints = gestures.wave();
  EXPECT_FALSE(waypoints.empty());
}
