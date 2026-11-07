#pragma once

#include <vector>

#include <Eigen/Dense>

#include "robotarm/arm_model.hpp"

namespace robotarm {

// Iterative damped-least-squares inverse kinematics: starting from
// initial_joint_values, nudges the joint values so the end effector's
// position moves toward target_position, using geometric_jacobian()'s
// linear (position) rows only - every arm this project models is a
// revolute/prismatic chain with no independent end-effector
// orientation control, so position is the only thing to solve for.
//
// Damped least squares (not a plain Jacobian pseudo-inverse)
// specifically because a plain pseudo-inverse blows up near a
// singularity - e.g. the arm fully extended, reaching for a point at
// its own maximum reach, where the Jacobian loses rank. The damping
// term keeps the step bounded there instead of dividing by a
// near-zero singular value.
//
// Returns the resulting joint values (same size as
// initial_joint_values, same units: radians for revolute, meters for
// prismatic) after at most max_iterations steps, or once the position
// error drops below 1e-6 meters, whichever comes first. Does not
// itself decide whether the result is "close enough" to use for an
// unreachable target - the caller can check that by running
// forward_kinematics() on the result and comparing to target_position
// (exactly what inverse_kinematics_test.cpp and ArmController's
// moveToTarget() both do).
//
// Each iteration's task-space error is capped at a quarter of the arm's
// total reach before solving. Without that cap, a target far outside the
// workspace produces a huge damped step that overshoots, so the arm can
// end up FARTHER from the target than where it started.
std::vector<double> solve_inverse_kinematics(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                                              const Eigen::Vector3d &target_position, int max_iterations = 200,
                                              double damping = 0.05, double step_scale = 1.0);

// Joint-limited variant: after every step each joint value is clamped
// into [min, max] (radians for revolute, meters for prismatic) - a
// projected damped-least-squares solve. `limits` must have one entry per
// joint, or be empty for "no limits".
struct JointLimit {
  double min = -1e9;
  double max = 1e9;
};

std::vector<double> solve_inverse_kinematics(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                                              const Eigen::Vector3d &target_position,
                                              const std::vector<JointLimit> &limits, int max_iterations = 200,
                                              double damping = 0.05, double step_scale = 1.0);

// ---- planar arms: position AND hand direction ----------------------------

struct PlanarIkResult {
  std::vector<double> joint_values;
  double position_error = 0.0; // m, |tip - target| at the result
  double angle_error = 0.0;    // rad, |hand angle - target angle| (0 when no angle was asked for)
  bool converged = false;      // both errors within tolerance
};

// Inverse kinematics for a planar arm (every joint turning about the same
// axis, alpha = 0) with any number of joints, solving for the tip's
// position in the arm's plane and - optionally - the direction the last
// link points (`hand_angle`, radians, measured like a joint angle: 0 =
// along the base's +x).
//
// Why a separate solver: the general one above only controls position.
// To grasp an upright bottle the hand has to arrive level (or pointing
// straight down for a gripper), so orientation is part of the task. For a
// planar arm that's one extra row in the Jacobian - every revolute joint
// turns the hand by exactly the joint's own rotation, so d(angle)/dq_i = 1.
//
// Joint limits are handled as an active set, not just by clamping: a
// joint pinned at a limit that the step would push further out is taken
// out of the solve for that step, and the step is recomputed with the
// joints that can still move. Plain clamping keeps "spending" the step on
// the pinned joint, so the solve crawls - the arm visibly lags behind a
// dragged target whenever an elbow is straight.
//
// Errors are weighted so 1 rad of hand angle counts like 0.1 m of position.
PlanarIkResult solve_planar_ik(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                               const Eigen::Vector2d &tip_target, const double *hand_angle,
                               const std::vector<JointLimit> &limits, int max_iterations = 150,
                               double damping = 0.02);

} // namespace robotarm
