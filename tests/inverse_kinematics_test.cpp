// Verifies solve_inverse_kinematics(): the real check throughout is
// "does forward_kinematics() of the result land back on the target",
// not any particular joint-value answer (IK can legitimately reach the
// same point from more than one elbow configuration) - plus the
// specific behaviors a damped-least-squares solver is supposed to have
// that a plain pseudo-inverse wouldn't: staying finite at a singularity,
// and terminating (not hanging) on a target the arm can't reach at all.
#include <cmath>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/inverse_kinematics.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::solve_inverse_kinematics;

namespace {

// config/two_link_planar.json's geometry (same convention every other
// test file in this project uses for the hand-built two-link arm).
ArmModel two_link_arm() { return ArmModel({DHParameter(1.0), DHParameter(0.5)}); }

Eigen::Vector3d tip_position(const ArmModel &arm, const std::vector<double> &joint_values) {
  const std::vector<Eigen::Matrix4d> frames = arm.forward_kinematics(joint_values);
  if (frames.empty()) return Eigen::Vector3d::Zero();
  return frames.back().block<3, 1>(0, 3);
}

} // namespace

TEST(InverseKinematics, ReachesAKnownReachableTarget) {
  const ArmModel arm = two_link_arm();
  const std::vector<double> known_pose = {0.4, -0.6};
  const Eigen::Vector3d target = tip_position(arm, known_pose);

  const std::vector<double> result = solve_inverse_kinematics(arm, {0.0, 0.0}, target);
  const Eigen::Vector3d reached = tip_position(arm, result);

  EXPECT_NEAR((reached - target).norm(), 0.0, 1e-3);
}

TEST(InverseKinematics, ReachesADifferentTargetFromADifferentStartingGuess) {
  // Same arm, a different target and a different starting guess - not
  // just re-running the same numbers with different variable names.
  const ArmModel arm = two_link_arm();
  const std::vector<double> known_pose = {-1.2, 1.5};
  const Eigen::Vector3d target = tip_position(arm, known_pose);

  const std::vector<double> result = solve_inverse_kinematics(arm, {1.0, -1.0}, target);
  const Eigen::Vector3d reached = tip_position(arm, result);

  EXPECT_NEAR((reached - target).norm(), 0.0, 1e-3);
}

TEST(InverseKinematics, StaysPutWhenAlreadyAtTheTarget) {
  const ArmModel arm = two_link_arm();
  const std::vector<double> pose = {0.3, 0.2};
  const Eigen::Vector3d target = tip_position(arm, pose);

  const std::vector<double> result = solve_inverse_kinematics(arm, pose, target);
  const Eigen::Vector3d reached = tip_position(arm, result);

  EXPECT_NEAR((reached - target).norm(), 0.0, 1e-6);
}

TEST(InverseKinematics, RemainsFiniteAtAFullyExtendedSingularity) {
  // The arm fully extended (both joints at 0) reaching for a point
  // exactly at its own maximum reach - the Jacobian loses rank here.
  // A plain pseudo-inverse would divide by a near-zero singular value;
  // damping must keep every step finite instead.
  const ArmModel arm = two_link_arm();
  const Eigen::Vector3d target(1.5, 0.0, 0.0); // exactly a=1.0 + a=0.5

  const std::vector<double> result = solve_inverse_kinematics(arm, {0.0, 0.0}, target);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_TRUE(std::isfinite(result[0]));
  EXPECT_TRUE(std::isfinite(result[1]));
  const Eigen::Vector3d reached = tip_position(arm, result);
  EXPECT_TRUE(std::isfinite(reached.x()));
  EXPECT_NEAR((reached - target).norm(), 0.0, 1e-2);
}

TEST(InverseKinematics, TerminatesAndStaysFiniteOnAnUnreachableTarget) {
  // Total reach is 1.5m - this target is nowhere close, so the solver
  // must still terminate (bounded by max_iterations, not an infinite
  // loop chasing an impossible target) and return a finite result
  // that's at least closer than where it started, not NaN/garbage.
  //
  // The starting pose matters here: {0,0} happens to already be the
  // single closest reachable point to a target straight out along +X
  // (the arm fully extended toward it) - there's genuinely no better
  // pose, so "stays exactly put" would be the CORRECT answer from
  // that start, not a bug. Starting folded up instead, pointing away
  // from the target, leaves real room to improve.
  const ArmModel arm = two_link_arm();
  const Eigen::Vector3d target(100.0, 0.0, 0.0);
  const std::vector<double> start = {1.5, 1.5};

  const std::vector<double> result = solve_inverse_kinematics(arm, start, target, 200);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_TRUE(std::isfinite(result[0]));
  EXPECT_TRUE(std::isfinite(result[1]));

  const double distance_before = (tip_position(arm, start) - target).norm();
  const double distance_after = (tip_position(arm, result) - target).norm();
  EXPECT_LT(distance_after, distance_before);
}

TEST(InverseKinematics, ResultHasOneEntryPerJoint) {
  const ArmModel arm = two_link_arm();
  const std::vector<double> result = solve_inverse_kinematics(arm, {0.0, 0.0}, Eigen::Vector3d(1.0, 0.2, 0.0));
  EXPECT_EQ(result.size(), arm.num_joints());
}
