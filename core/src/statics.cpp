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
  const std::vector<Eigen::Matrix4d> poses = arm.forward_kinematics(joint_values);

  // Every weight and where it acts, in the base frame: each link's at its
  // center of mass, the payload's at the end-effector.
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> forces;
  std::vector<size_t> carried_by; // the last joint that moves the point
  for (size_t k = 1; k <= n; k++) {
    if (links[k - 1].mass == 0.0) continue;
    const Eigen::Matrix4d &frame = poses[k - 1];
    points.push_back(frame.block<3, 1>(0, 3) + frame.block<3, 3>(0, 0) * links[k - 1].center_of_mass);
    forces.push_back(links[k - 1].mass * gravity);
    carried_by.push_back(k);
  }
  if (payload_mass != 0.0) {
    points.push_back(poses[n - 1].block<3, 1>(0, 3));
    forces.push_back(payload_mass * gravity);
    carried_by.push_back(n);
  }

  // Jacobian transpose, one column at a time: joint i feels every weight
  // on a link it moves (k >= i).
  Eigen::VectorXd load_moment = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));
  for (size_t i = 1; i <= n; i++) {
    const Eigen::Matrix4d frame_before = (i == 1) ? Eigen::Matrix4d::Identity() : poses[i - 2];
    const Eigen::Vector3d z_axis = frame_before.block<3, 1>(0, 2);
    const Eigen::Vector3d origin_before = frame_before.block<3, 1>(0, 3);
    const bool revolute = arm.joints()[i - 1].type == JointType::Revolute;

    double moment = 0.0;
    for (size_t w = 0; w < points.size(); w++) {
      if (carried_by[w] < i) continue;
      moment += revolute ? z_axis.dot((points[w] - origin_before).cross(forces[w])) : z_axis.dot(forces[w]);
    }
    load_moment(static_cast<Eigen::Index>(i - 1)) = moment;
  }

  // The actuator must supply the opposite of the load's own moment to hold
  // equilibrium - see the sign-convention note in statics.hpp.
  return -load_moment;
}

} // namespace robotarm
