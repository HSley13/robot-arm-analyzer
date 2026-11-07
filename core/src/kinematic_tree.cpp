#include "robotarm/kinematic_tree.hpp"

#include <cassert>
#include <set>
#include <stdexcept>

#include "robotarm/units.hpp"

namespace robotarm {

Eigen::Matrix4d xyz_rpy_transform(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy_radians) {
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(rpy_radians.z(), Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(rpy_radians.y(), Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(rpy_radians.x(), Eigen::Vector3d::UnitX()))
                                       .toRotationMatrix();
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = xyz;
  return transform;
}

KinematicTree::KinematicTree(std::vector<KinematicChain> chains) : _chains(std::move(chains)) {
  if (_chains.empty()) {
    throw std::invalid_argument("KinematicTree: must have at least one chain");
  }

  std::set<std::string> resolved;
  for (const KinematicChain &chain : _chains) {
    if (chain.name.empty()) {
      throw std::invalid_argument("KinematicTree: a chain has an empty name");
    }
    if (resolved.count(chain.name) > 0) {
      throw std::invalid_argument("KinematicTree: duplicate chain name '" + chain.name + "'");
    }
    if (!chain.parent_chain.empty() && resolved.count(chain.parent_chain) == 0) {
      throw std::invalid_argument("KinematicTree: chain '" + chain.name + "' has parent_chain '" +
                                   chain.parent_chain + "', which must appear earlier in the chain list");
    }
    resolved.insert(chain.name);
  }
}

const KinematicChain &KinematicTree::chain(const std::string &name) const {
  for (const KinematicChain &c : _chains) {
    if (c.name == name) return c;
  }
  throw std::out_of_range("KinematicTree: no chain named '" + name + "'");
}

std::map<std::string, std::vector<Eigen::Matrix4d>> KinematicTree::forward_kinematics(
    const std::map<std::string, std::vector<double>> &joint_values) const {
  std::map<std::string, Eigen::Matrix4d> chain_tip; // world-frame tip transform per chain
  std::map<std::string, std::vector<Eigen::Matrix4d>> result;

  for (const KinematicChain &chain : _chains) {
    const Eigen::Matrix4d parent_tip =
        chain.parent_chain.empty() ? Eigen::Matrix4d::Identity() : chain_tip.at(chain.parent_chain);
    const Eigen::Matrix4d base = parent_tip * chain.origin;

    const std::vector<double> &values = joint_values.at(chain.name);
    assert(values.size() == chain.joints.size());

    std::vector<Eigen::Matrix4d> poses;
    poses.reserve(chain.joints.size());
    Eigen::Matrix4d cumulative = base;
    for (size_t i = 0; i < chain.joints.size(); i++) {
      const DHParameter &joint = chain.joints[i];
      double theta = joint.theta;
      double d = joint.d;
      if (joint.type == JointType::Revolute) {
        theta = normalize_angle(theta + values[i]);
      } else {
        d += values[i];
      }
      cumulative = cumulative * dh_transform(theta, d, joint.a, joint.alpha);
      poses.push_back(cumulative);
    }

    chain_tip[chain.name] = poses.empty() ? base : poses.back();
    result[chain.name] = std::move(poses);
  }

  return result;
}

Eigen::Matrix4d KinematicTree::chain_base_frame(
    const std::string &name, const std::map<std::string, std::vector<Eigen::Matrix4d>> &poses) const {
  const KinematicChain &c = chain(name);
  Eigen::Matrix4d parent_tip = Eigen::Matrix4d::Identity();
  if (!c.parent_chain.empty()) {
    const auto it = poses.find(c.parent_chain);
    if (it != poses.end() && !it->second.empty()) {
      parent_tip = it->second.back();
    } else {
      // A joint-less parent's tip is its own base frame.
      parent_tip = chain_base_frame(c.parent_chain, poses);
    }
  }
  return parent_tip * c.origin;
}

} // namespace robotarm
