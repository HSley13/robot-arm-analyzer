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

TEST(InverseKinematics, StableNearTheReachBoundaryForTwoVeryCloseTargets) {
  // The entire reach boundary (not just one special pose) is a
  // singularity for a serial chain - the arm is always fully extended
  // there. Dragging a UI target along that boundary calls this solver
  // repeatedly with targets that are only a tiny bit apart; without an
  // early-exit once the solver's own proposed step stops being
  // meaningful (see the production code's comment), every one of
  // those calls burns the full max_iterations budget bouncing around
  // a residual radial error it can never close, and two calls that
  // differ only by a fraction of a millimeter can settle on
  // noticeably different final poses - exactly the "lags, feels
  // unstable" symptom reported from actually using it. Two targets
  // 0.1mm apart, from the SAME starting pose (isolating the solver's
  // own behavior from whatever pose a previous UI frame left behind),
  // must produce two nearly-identical poses, not a visible jump.
  const ArmModel arm = two_link_arm();
  const std::vector<double> start = {0.1, 0.2}; // not already sitting exactly on the boundary
  const Eigen::Vector3d target_a(1.5, 0.0, 0.0); // exactly at max reach (a=1.0 + a=0.5)
  const Eigen::Vector3d target_b(1.4999, 0.0001, 0.0); // 0.1mm away along the same boundary

  const std::vector<double> result_a = solve_inverse_kinematics(arm, start, target_a);
  const std::vector<double> result_b = solve_inverse_kinematics(arm, start, target_b);

  // Exactly AT the boundary (target_a) is a true singularity - a damped
  // solver trades some precision there for staying finite/bounded at
  // all (its whole documented purpose), so a sub-degree residual
  // spread between these two is an inherent, acceptable property of
  // that tradeoff, not a bug. What this guards against is the much
  // larger, visually-obvious swing an UN-bounded 200-iteration grind
  // can produce once it starts oscillating around an irreducible
  // residual - this tolerance is comfortably above the measured,
  // damping-inherent spread and comfortably below "visibly jumps".
  ASSERT_EQ(result_a.size(), result_b.size());
  for (size_t i = 0; i < result_a.size(); i++) {
    EXPECT_NEAR(result_a[i], result_b[i], 0.02);
  }
}

TEST(InverseKinematics, ResultHasOneEntryPerJoint) {
  const ArmModel arm = two_link_arm();
  const std::vector<double> result = solve_inverse_kinematics(arm, {0.0, 0.0}, Eigen::Vector3d(1.0, 0.2, 0.0));
  EXPECT_EQ(result.size(), arm.num_joints());
}

TEST(InverseKinematics, RespectsJointLimits) {
  // An "elbow" that can only flex positively: a target reachable only by
  // bending it negatively must leave it at (or inside) its limit.
  const ArmModel arm = two_link_arm();
  const std::vector<robotarm::JointLimit> limits = {{-3.14159265358979323846, 3.14159265358979323846}, {0.0, 2.5}};
  const Eigen::Vector3d target(0.9, -0.6, 0.0);

  const std::vector<double> result = solve_inverse_kinematics(arm, {0.3, 0.3}, target, limits);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_GE(result[1], 0.0 - 1e-12);
  EXPECT_LE(result[1], 2.5 + 1e-12);
  // Still reaches it, using the other elbow solution.
  EXPECT_LT((tip_position(arm, result) - target).norm(), 1e-3);
}
