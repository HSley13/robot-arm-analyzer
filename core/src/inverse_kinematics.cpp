#include "robotarm/inverse_kinematics.hpp"

#include "robotarm/statics.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

std::vector<double> solve_inverse_kinematics(const ArmModel &arm, const std::vector<double> &initial_joint_values,
                                              const Eigen::Vector3d &target_position, int max_iterations,
                                              double damping, double step_scale) {
  const size_t n = arm.num_joints();
  std::vector<double> q = initial_joint_values;

  for (int iter = 0; iter < max_iterations; iter++) {
    const std::vector<Eigen::Matrix4d> frames = arm.forward_kinematics(q);
    Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
    if (!frames.empty()) current_position = frames.back().block<3, 1>(0, 3);

    const Eigen::Vector3d error = target_position - current_position;
    if (error.norm() < 1e-6) break;

    const Eigen::MatrixXd jacobian = geometric_jacobian(arm, q, n).topRows(3); // position rows only

    // Damped least squares: dq = J^T * (J*J^T + damping^2 * I)^-1 * error -
    // see the header's doc comment for why this is damped, not a
    // plain pseudo-inverse.
    const Eigen::Matrix3d jjt = jacobian * jacobian.transpose();
    const Eigen::Matrix3d damped = jjt + (damping * damping) * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq = jacobian.transpose() * damped.ldlt().solve(error);

    for (size_t i = 0; i < n; i++) {
      q[i] += step_scale * dq(static_cast<Eigen::Index>(i));
      if (arm.joints()[i].type == JointType::Revolute) q[i] = normalize_angle(q[i]);
    }
  }

  return q;
}

} // namespace robotarm
