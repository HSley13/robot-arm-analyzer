#include "robotarm/arm_model.hpp"

#include <cassert>
#include <utility>

namespace robotarm {

ArmModel::ArmModel(std::vector<DHParameter> joints) : joints_(std::move(joints)) {
  assert(!joints_.empty());
}

std::vector<Eigen::Matrix4d> ArmModel::forward_kinematics(
    const std::vector<double>& joint_values) const {
  assert(joint_values.size() == joints_.size());

  std::vector<Eigen::Matrix4d> poses;
  poses.reserve(joints_.size());

  Eigen::Matrix4d cumulative = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const DHParameter& joint = joints_[i];
    double theta = joint.theta;
    double d = joint.d;
    if (joint.type == JointType::Revolute) {
      theta += joint_values[i];
    } else {
      d += joint_values[i];
    }
    cumulative = cumulative * dh_transform(theta, d, joint.a, joint.alpha);
    poses.push_back(cumulative);
  }
  return poses;
}

Eigen::Matrix4d ArmModel::end_effector_pose(
    const std::vector<double>& joint_values) const {
  return forward_kinematics(joint_values).back();
}

}  // namespace robotarm
