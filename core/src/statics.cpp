#include "robotarm/statics.hpp"

#include <cassert>

namespace robotarm {

Eigen::MatrixXd geometric_jacobian(const ArmModel &arm, const std::vector<double> &joint_values, size_t target_joint) {
  assert(target_joint >= 1 && target_joint <= arm.num_joints());

  const std::vector<Eigen::Matrix4d> poses = arm.forward_kinematics(joint_values);
  const size_t n = arm.num_joints();
  const Eigen::Vector3d target_origin = poses[target_joint - 1].block<3, 1>(0, 3);

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, n);
  for (size_t i = 1; i <= target_joint; i++) {
    const Eigen::Matrix4d frame_before = (i == 1) ? Eigen::Matrix4d::Identity() : poses[i - 2];
    const Eigen::Vector3d z_axis = frame_before.block<3, 1>(0, 2);
    const Eigen::Vector3d origin_before = frame_before.block<3, 1>(0, 3);

    Eigen::Vector3d jv;
    Eigen::Vector3d jw;
    if (arm.joints()[i - 1].type == JointType::Revolute) {
      jv = z_axis.cross(target_origin - origin_before);
      jw = z_axis;
    } else {
      jv = z_axis;
      jw = Eigen::Vector3d::Zero();
    }
    jacobian.block<3, 1>(0, i - 1) = jv;
    jacobian.block<3, 1>(3, i - 1) = jw;
  }
  return jacobian;
}

Eigen::VectorXd static_joint_torques(const ArmModel &arm, const std::vector<double> &joint_values, const std::vector<LinkMass> &links, double payload_mass, const Eigen::Vector3d &gravity) {
  assert(links.size() == arm.num_joints());
  const size_t n = arm.num_joints();

  Eigen::VectorXd load_moment = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);

  for (size_t k = 1; k <= n; k++) {
    if (links[k - 1].mass == 0.0) continue;

    wrench.head<3>() = links[k - 1].mass * gravity;
    load_moment += geometric_jacobian(arm, joint_values, k).transpose() * wrench;
  }

  if (payload_mass != 0.0) {
    wrench.head<3>() = payload_mass * gravity;
    load_moment += geometric_jacobian(arm, joint_values, n).transpose() * wrench;
  }

  // The actuator must supply the opposite of the load's own moment to hold
  // equilibrium - see the sign-convention note in statics.hpp.
  return -load_moment;
}

} // namespace robotarm
