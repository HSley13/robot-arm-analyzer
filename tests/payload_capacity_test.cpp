// Verifies max_safe_payload(): the synthetic-evaluator cases pin down
// exactly what the binary search is supposed to do (pick the binding
// joint's own breakeven point, handle an already-overloaded pose, and
// saturate at the search ceiling rather than guessing past it), then a
// real-physics case cross-checks the result against static_joint_torques()
// directly - the actual point of the function is to drive a real torque
// evaluator, not just an abstract one.
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "robotarm/arm_config.hpp"
#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/dynamics.hpp"
#include "robotarm/payload_capacity.hpp"
#include "robotarm/statics.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::LinkMass;
using robotarm::max_safe_payload;
using robotarm::max_safe_payload_linear;
using robotarm::static_joint_torques;

TEST(PayloadCapacity, PicksTheBindingJointsOwnBreakevenPoint) {
  // torque(payload) = base + payload * slope, per joint - a linear
  // evaluator makes the expected answer computable by hand.
  const std::vector<double> base = {2.0, 3.0};
  const std::vector<double> slope = {1.0, 0.5};
  const std::vector<double> limits = {10.0, 10.0};

  auto evaluator = [&](double payload) {
    Eigen::VectorXd tau(2);
    tau(0) = base[0] + payload * slope[0];
    tau(1) = base[1] + payload * slope[1];
    return tau;
  };

  // Joint 0 hits its limit at payload=(10-2)/1=8; joint 1 at
  // (10-3)/0.5=14 - joint 0 binds first.
  const double result = max_safe_payload(evaluator, limits, 1000.0, 1e-6);
  EXPECT_NEAR(result, 8.0, 1e-3);
}

TEST(PayloadCapacity, SwitchesWhichJointBindsBasedOnTheSlopes) {
  // Same shape, but now joint 1 is the one that binds first.
  const std::vector<double> base = {1.0, 1.0};
  const std::vector<double> slope = {0.2, 2.0};
  const std::vector<double> limits = {10.0, 10.0};

  auto evaluator = [&](double payload) {
    Eigen::VectorXd tau(2);
    tau(0) = base[0] + payload * slope[0];
    tau(1) = base[1] + payload * slope[1];
    return tau;
  };

  // Joint 0: (10-1)/0.2=45; joint 1: (10-1)/2=4.5 - joint 1 binds.
  const double result = max_safe_payload(evaluator, limits, 1000.0, 1e-6);
  EXPECT_NEAR(result, 4.5, 1e-3);
}

TEST(PayloadCapacity, ReturnsZeroWhenAlreadyOverloadedAtZeroPayload) {
  const std::vector<double> limits = {10.0};
  auto evaluator = [&](double) {
    Eigen::VectorXd tau(1);
    tau(0) = 50.0; // already past the limit, independent of payload
    return tau;
  };

  EXPECT_DOUBLE_EQ(max_safe_payload(evaluator, limits), 0.0);
}

TEST(PayloadCapacity, SaturatesAtTheSearchCeilingRatherThanGuessingPastIt) {
  const std::vector<double> limits = {1000.0};
  auto evaluator = [&](double) {
    Eigen::VectorXd tau(1);
    tau(0) = 1.0; // never gets anywhere near the limit, any payload
    return tau;
  };

  EXPECT_DOUBLE_EQ(max_safe_payload(evaluator, limits, 50.0), 50.0);
}

TEST(PayloadCapacity, IgnoresJointsWithANonPositiveLimit) {
  const std::vector<double> limits = {0.0, 10.0};
  auto evaluator = [&](double payload) {
    Eigen::VectorXd tau(2);
    tau(0) = payload * 1000.0; // would "fail" any positive limit - ignored since limit is 0
    tau(1) = payload * 1.0;
    return tau;
  };

  EXPECT_NEAR(max_safe_payload(evaluator, limits, 1000.0, 1e-6), 10.0, 1e-3);
}

TEST(PayloadCapacity, MatchesStaticJointTorquesDirectlyOnARealArm) {
  // config/two_link_planar.json's geometry, hand-built (same convention
  // surrogate_test.cpp and dynamic_surrogate_test.cpp already use).
  const ArmModel arm({DHParameter(1.0), DHParameter(0.5)});
  const std::vector<LinkMass> links = {LinkMass(2.0), LinkMass(1.0)};
  const Eigen::Vector3d gravity_down_y(0.0, -9.81, 0.0);
  // theta=0 points the first link along local/world X - perpendicular
  // to this gravity vector, i.e. "arm extended horizontally", the
  // worst case for gravity loading. (theta=90 would point it ALONG
  // gravity instead, where the moment arm - and so the torque - is
  // ~0 regardless of payload; not the case this test wants.)
  const std::vector<double> joint_values = {0.0, 0.0};
  const std::vector<double> limits = {50.0, 25.0};

  auto evaluator = [&](double payload) {
    return static_joint_torques(arm, joint_values, links, payload, gravity_down_y);
  };

  const double result = max_safe_payload(evaluator, limits, 200.0, 1e-4);

  // The result must be a real breakeven point: right at it, the
  // binding joint's ratio is ~1.0, and a little past it, some joint's
  // ratio exceeds 1.0 - not just "some number the search returned".
  const Eigen::VectorXd at_result = evaluator(result);
  double max_ratio_at_result = 0.0;
  for (int i = 0; i < 2; i++) max_ratio_at_result = std::max(max_ratio_at_result, std::abs(at_result(i)) / limits[static_cast<size_t>(i)]);
  EXPECT_NEAR(max_ratio_at_result, 1.0, 1e-2);

  const Eigen::VectorXd just_past = evaluator(result + 1.0);
  double max_ratio_just_past = 0.0;
  for (int i = 0; i < 2; i++) max_ratio_just_past = std::max(max_ratio_just_past, std::abs(just_past(i)) / limits[static_cast<size_t>(i)]);
  EXPECT_GT(max_ratio_just_past, 1.0);
}

TEST(PayloadCapacity, ClosedFormHandlesEverySignOfLoad) {
  // Joint 0 loaded the "negative" way (payload pushes it further from
  // zero below), joint 1 helped by the payload at first (it starts at -6
  // and the payload pulls it through zero toward +10).
  const Eigen::VectorXd at_zero = (Eigen::VectorXd(2) << -2.0, -6.0).finished();
  const Eigen::VectorXd per_kg = (Eigen::VectorXd(2) << -1.0, 1.0).finished();
  const std::vector<double> limits = {20.0, 10.0};
  // Joint 0: -2 - m >= -20 -> m <= 18. Joint 1: -6 + m <= 10 -> m <= 16.
  EXPECT_DOUBLE_EQ(max_safe_payload_linear(at_zero, per_kg, limits), 16.0);
  // A joint the payload doesn't load never binds; nothing binds -> ceiling.
  EXPECT_DOUBLE_EQ(max_safe_payload_linear(at_zero, Eigen::VectorXd::Zero(2), limits, 50.0), 50.0);
  // Already overloaded with nothing in the hand.
  EXPECT_DOUBLE_EQ(max_safe_payload_linear((Eigen::VectorXd(2) << 25.0, 0.0).finished(), per_kg, limits), 0.0);
}

TEST(PayloadCapacity, ClosedFormMatchesTheSearchOnTheRealArm) {
  // Static and dynamic torques of the shipped arm are exactly linear in
  // the payload, so the closed form and the bisection must agree, at
  // every pose and motion.
  const robotarm::ArmConfig config =
      robotarm::load_arm_config(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json");
  const ArmModel arm = robotarm::to_arm_model(config);
  const std::vector<LinkMass> links = robotarm::to_link_masses(config);
  const std::vector<robotarm::LinkInertial> inertials = robotarm::to_link_inertials(config);
  std::vector<double> limits;
  for (const robotarm::JointConfig &joint : config.joints) limits.push_back(joint.torque_limit);
  const Eigen::Vector3d gravity(0.0, -9.81, 0.0);

  const std::vector<std::vector<double>> poses = {{0.0, 0.0, 0.0}, {-0.6, 1.4, -0.2}, {-1.57, 0.0, 0.0},
                                                  {1.2, 0.5, 0.8}, {-2.0, 2.2, -0.9}};
  const std::vector<double> velocity = {1.2, -0.8, 2.0};
  const std::vector<double> acceleration = {-3.0, 6.0, 4.0};
  for (const std::vector<double> &pose : poses) {
    auto statics = [&](double payload) { return static_joint_torques(arm, pose, links, payload, gravity); };
    auto dynamics = [&](double payload) {
      return robotarm::inverse_dynamics(arm, pose, velocity, acceleration, inertials, payload, gravity);
    };
    EXPECT_NEAR(max_safe_payload_linear(statics, limits, 200.0), max_safe_payload(statics, limits, 200.0, 1e-6), 1e-5);
    EXPECT_NEAR(max_safe_payload_linear(dynamics, limits, 200.0), max_safe_payload(dynamics, limits, 200.0, 1e-6),
                1e-5);
  }
}
