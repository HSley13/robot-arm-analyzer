#include "robotarm/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "robotarm/statics.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

std::vector<double> solve_inverse_kinematics(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                                              const Eigen::Vector3d &target_position, int max_iterations,
                                              double damping, double step_scale) {
  return solve_inverse_kinematics(arm, initial_joint_values, target_position, {}, max_iterations, damping, step_scale);
}

std::vector<double> solve_inverse_kinematics(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                                              const Eigen::Vector3d &target_position,
                                              const std::vector<JointLimit> &limits, int max_iterations,
                                              double damping, double step_scale) {
  const size_t n = arm.num_joints();
  std::vector<double> q = initial_joint_values;
  const bool limited = limits.size() == n;

  double total_reach = 0.0;
  for (const DHParameter &joint : arm.joints()) {
    total_reach += std::abs(joint.a) + std::abs(joint.d);
  }
  const double max_error_step = std::max(1e-3, 0.25 * total_reach);

  for (int iter = 0; iter < max_iterations; iter++) {
    const std::vector<Eigen::Matrix4d> frames = arm.forward_kinematics(q);
    Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
    if (!frames.empty()) current_position = frames.back().block<3, 1>(0, 3);

    Eigen::Vector3d error = target_position - current_position;
    if (error.norm() < 1e-6) break;
    if (error.norm() > max_error_step) {
      error *= max_error_step / error.norm();
    }

    const Eigen::MatrixXd jacobian = geometric_jacobian(arm, q, n).topRows(3); // position rows only

    // Damped least squares: dq = J^T * (J*J^T + damping^2 * I)^-1 * error -
    // see the header's doc comment for why this is damped, not a
    // plain pseudo-inverse.
    const Eigen::Matrix3d jjt = jacobian * jacobian.transpose();
    const Eigen::Matrix3d damped = jjt + (damping * damping) * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq = jacobian.transpose() * damped.ldlt().solve(error);

    // Near a singularity (the arm at its own maximum reach - ALWAYS a
    // singular configuration for a serial chain, continuously along
    // the entire reach boundary, not just one special pose) the
    // damped solver correctly assigns near-zero gain to the direction
    // it structurally can't move in, but error.norm() above can stay
    // stuck well above 1e-6 forever if that unreachable component
    // never shrinks. Without this check, every call for a target at
    // or beyond the boundary burns the full max_iterations budget
    // doing nothing - slow (felt as visible lag while dragging a
    // target along the boundary) and a source of run-to-run
    // instability (200 iterations of a near-singular solve bouncing
    // around a residual that can't close don't reliably settle on the
    // same pose for two very close nearby targets). Breaking as soon
    // as the solver's own proposed step stops being meaningful - not
    // just when the position error is small - fixes both.
    if (dq.norm() < 1e-8) break;

    for (size_t i = 0; i < n; i++) {
      q[i] += step_scale * dq(static_cast<Eigen::Index>(i));
      if (arm.joints()[i].type == JointType::Revolute) q[i] = normalize_angle(q[i]);
      if (limited) {
        q[i] = std::clamp(q[i], limits[i].min, limits[i].max);
      }
    }
  }

  return q;
}

PlanarIkResult solve_planar_ik(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                               const Eigen::Vector2d &tip_target, const double *hand_angle,
                               const std::vector<JointLimit> &limits, int max_iterations, double damping) {
  constexpr double kAngleWeight = 0.1; // m per rad
  constexpr double kPositionTolerance = 1e-4;
  constexpr double kAngleTolerance = 1e-3;

  const size_t n = arm.num_joints();
  const bool limited = limits.size() == n;
  const Eigen::Index rows = hand_angle != nullptr ? 3 : 2;

  double total_reach = 0.0;
  for (const DHParameter &joint : arm.joints()) {
    total_reach += std::abs(joint.a) + std::abs(joint.d);
  }
  const double max_position_step = std::max(1e-3, 0.25 * total_reach);

  PlanarIkResult result;
  result.joint_values = initial_joint_values;
  std::vector<double> &q = result.joint_values;
  if (limited) {
    for (size_t i = 0; i < n; i++) {
      q[i] = std::clamp(q[i], limits[i].min, limits[i].max);
    }
  }

  auto evaluate = [&](Eigen::VectorXd &error) {
    const Eigen::Matrix4d tip = arm.forward_kinematics(q).back();
    const Eigen::Vector2d position(tip(0, 3), tip(1, 3));
    result.position_error = (tip_target - position).norm();
    error.resize(rows);
    error.head<2>() = tip_target - position;
    result.angle_error = 0.0;
    if (hand_angle != nullptr) {
      const double angle = std::atan2(tip(1, 0), tip(0, 0));
      const double angle_error = normalize_angle(*hand_angle - angle);
      result.angle_error = std::abs(angle_error);
      error(2) = kAngleWeight * angle_error;
    }
  };

  Eigen::VectorXd error;
  for (int iter = 0; iter < max_iterations; iter++) {
    evaluate(error);
    if (result.position_error < kPositionTolerance && result.angle_error < kAngleTolerance) {
      result.converged = true;
      return result;
    }
    if (error.head<2>().norm() > max_position_step) {
      error.head<2>() *= max_position_step / error.head<2>().norm();
    }
    if (rows == 3) {
      error(2) = std::clamp(error(2), -0.05, 0.05);
    }

    // Planar Jacobian: x, y rows of the geometric Jacobian plus the
    // rotation-about-z row (1 for every revolute joint).
    const Eigen::MatrixXd full = geometric_jacobian(arm, q, n);
    Eigen::MatrixXd jacobian(rows, static_cast<Eigen::Index>(n));
    jacobian.row(0) = full.row(0);
    jacobian.row(1) = full.row(1);
    if (rows == 3) {
      jacobian.row(2) = kAngleWeight * full.row(5);
    }

    // Active set: drop joints pinned at a limit that the step pushes
    // further out, and re-solve with the rest.
    std::vector<bool> active(n, true);
    Eigen::VectorXd dq;
    for (size_t pass = 0; pass <= n; pass++) {
      Eigen::MatrixXd j_active = jacobian;
      for (size_t i = 0; i < n; i++) {
        if (!active[i]) {
          j_active.col(static_cast<Eigen::Index>(i)).setZero();
        }
      }
      const Eigen::MatrixXd damped = j_active * j_active.transpose() +
                                     (damping * damping) * Eigen::MatrixXd::Identity(rows, rows);
      dq = j_active.transpose() * damped.ldlt().solve(error);

      bool changed = false;
      if (limited) {
        for (size_t i = 0; i < n; i++) {
          const double step = dq(static_cast<Eigen::Index>(i));
          const bool at_min = q[i] <= limits[i].min + 1e-9 && step < 0.0;
          const bool at_max = q[i] >= limits[i].max - 1e-9 && step > 0.0;
          if (active[i] && (at_min || at_max)) {
            active[i] = false;
            changed = true;
          }
        }
      }
      if (!changed) {
        break;
      }
    }
    if (dq.norm() < 1e-10) {
      break; // nothing can move any closer
    }

    for (size_t i = 0; i < n; i++) {
      q[i] += dq(static_cast<Eigen::Index>(i));
      if (arm.joints()[i].type == JointType::Revolute) {
        q[i] = normalize_angle(q[i]);
      }
      if (limited) {
        q[i] = std::clamp(q[i], limits[i].min, limits[i].max);
      }
    }
  }

  evaluate(error);
  result.converged = result.position_error < kPositionTolerance && result.angle_error < kAngleTolerance;
  return result;
}

} // namespace robotarm
