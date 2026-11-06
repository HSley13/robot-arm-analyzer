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

} // namespace robotarm
