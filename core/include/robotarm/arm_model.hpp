#pragma once

#include <vector>

#include <Eigen/Dense>

#include "robotarm/dh.hpp"

namespace robotarm {

// A serial-link arm described purely by a DH parameter table, so any
// serial-link robot (2-joint desk arm, 6-joint industrial arm, ...) is just a
// different table, not a different code path.
class ArmModel {
 public:
  explicit ArmModel(std::vector<DHParameter> joints);

  size_t num_joints() const { return joints_.size(); }
  const std::vector<DHParameter>& joints() const { return joints_; }

  // joint_values[i] is added to joint i's variable DH parameter (theta for
  // Revolute, d for Prismatic). Must have exactly one entry per joint.
  // Returns T_i^0 for i = 1..n: each joint's frame pose in the base frame,
  // in base-to-tip order.
  std::vector<Eigen::Matrix4d> forward_kinematics(
      const std::vector<double>& joint_values) const;

  // T_n^0: the end-effector frame pose in the base frame.
  Eigen::Matrix4d end_effector_pose(const std::vector<double>& joint_values) const;

 private:
  std::vector<DHParameter> joints_;
};

}  // namespace robotarm
