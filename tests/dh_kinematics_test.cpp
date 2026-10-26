// Verifies DH-based forward kinematics: identity/rotation/twist sanity
// checks, plus the textbook verification the roadmap calls for - a planar
// 2-link arm's end-effector position checked against the classic trig
// formula (x = sum L_k cos(theta_1+...+theta_k), y analogous).
#include <cmath>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::JointType;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;
} // namespace

TEST(DhKinematics, IdentityTransform) {
  ArmModel arm({DHParameter()});
  const std::vector<Eigen::Matrix4d> poses = arm.forward_kinematics({0.0});
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_TRUE(poses[0].isApprox(Eigen::Matrix4d::Identity(), kTol));
}

TEST(DhKinematics, PureRotation) {
  ArmModel arm({DHParameter()});
  const Eigen::Matrix4d t = arm.end_effector_pose({kPi / 2.0});
  EXPECT_NEAR(t(0, 0), 0.0, kTol);
  EXPECT_NEAR(t(1, 0), 1.0, kTol);
  EXPECT_NEAR(t(0, 1), -1.0, kTol);
  EXPECT_NEAR(t(1, 1), 0.0, kTol);
}

TEST(DhKinematics, PlanarTwoLinkMatchesTrig) {
  const double l1 = 1.0;
  const double l2 = 0.5;
  ArmModel arm({DHParameter(l1), DHParameter(l2)});

  const double theta1 = 30.0 * kPi / 180.0;
  const double theta2 = 45.0 * kPi / 180.0;
  const Eigen::Matrix4d t = arm.end_effector_pose({theta1, theta2});

  const double expected_x = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
  const double expected_y = l1 * std::sin(theta1) + l2 * std::sin(theta1 + theta2);

  EXPECT_NEAR(t(0, 3), expected_x, kTol);
  EXPECT_NEAR(t(1, 3), expected_y, kTol);
  EXPECT_NEAR(t(2, 3), 0.0, kTol);
}

TEST(DhKinematics, LinkTwist) {
  // a=1, alpha=90deg, theta=0: origin shifts by `a` along local x, and the
  // y-axis rotates into z (confirms the arm isn't limited to a flat plane).
  ArmModel arm({DHParameter(1.0, kPi / 2.0)});
  const Eigen::Matrix4d t = arm.end_effector_pose({0.0});
  EXPECT_NEAR(t(0, 3), 1.0, kTol);
  EXPECT_NEAR(t(1, 1), 0.0, kTol);
  EXPECT_NEAR(t(2, 1), 1.0, kTol);
}

TEST(DhKinematics, PrismaticJoint) {
  ArmModel arm({DHParameter(0.0, 0.0, 0.0, 0.0, JointType::Prismatic)});
  const Eigen::Matrix4d t = arm.end_effector_pose({2.0});
  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
  expected(2, 3) = 2.0;
  EXPECT_TRUE(t.isApprox(expected, kTol));
}

TEST(DhKinematics, RevoluteAngleDriftNormalizesToSamePose) {
  // A joint value of 10deg and one that has drifted by many full turns
  // (e.g. an integrated velocity over a long run) must produce the exact
  // same pose - normalize_angle() is applied before the trig, not after.
  ArmModel arm({DHParameter(1.0)});
  const double ten_degrees = 10.0 * kPi / 180.0;
  const double drifted = ten_degrees + 50.0 * 2.0 * kPi;

  const Eigen::Matrix4d t_clean = arm.end_effector_pose({ten_degrees});
  const Eigen::Matrix4d t_drifted = arm.end_effector_pose({drifted});

  EXPECT_TRUE(t_clean.isApprox(t_drifted, kTol));
}
