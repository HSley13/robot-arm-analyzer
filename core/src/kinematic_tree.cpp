#include "robotarm/kinematic_tree.hpp"

#include <cassert>
#include <set>
#include <stdexcept>

#include "robotarm/units.hpp"

namespace robotarm {

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
    const Eigen::Matrix4d base = chain.parent_chain.empty() ? Eigen::Matrix4d::Identity() : chain_tip.at(chain.parent_chain);

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

} // namespace robotarm
