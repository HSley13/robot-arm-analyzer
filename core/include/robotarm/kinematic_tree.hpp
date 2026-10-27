#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "robotarm/dh.hpp"

namespace robotarm {

// One branch of a tree-structured kinematic model - e.g. one finger of a
// multi-finger hand. Its own base frame is either the world origin
// (parent_chain empty) or the TIP frame of another named chain in the
// same KinematicTree (parent_chain set to that chain's name) - letting
// several chains share one attachment point. That's the genuinely
// different topology from a single serial chain the roadmap calls for
// multi-finger kinematics to get its own module rather than being folded
// into ArmModel.
struct KinematicChain {
  std::string name;
  std::string parent_chain; // empty => attaches to the world origin
  std::vector<DHParameter> joints;
};

// A tree of KinematicChains, each described purely by data - the same
// generality goal ArmModel has for a single chain, extended to branching
// structures. Chains must be given in an order where each chain's parent
// (if any) already appears earlier in the list - a simple, explicit
// ordering contract rather than a general topological sort, which this
// stretch-goal module doesn't need.
class KinematicTree {
public:
  // Throws std::invalid_argument if chains is empty, a name is empty or
  // repeated, or a parent_chain doesn't match an earlier chain's name -
  // fail loudly on a malformed tree rather than silently misplacing a
  // branch.
  explicit KinematicTree(std::vector<KinematicChain> chains);

  size_t num_chains() const { return _chains.size(); }
  const std::vector<KinematicChain> &chains() const { return _chains; }
  const KinematicChain &chain(const std::string &name) const;

  // joint_values: one entry per chain, keyed by chain name, each the
  // same length as that chain's own joints (same per-chain convention as
  // ArmModel::forward_kinematics - radians for Revolute, meters for
  // Prismatic, added to that joint's fixed DH offset).
  //
  // Returns, per chain, the same T_i^0-per-joint poses
  // ArmModel::forward_kinematics() returns for a single chain - but here
  // "0" always means the WORLD base frame, with the parent chain's tip
  // transform already folded in for any chain that has one.
  std::map<std::string, std::vector<Eigen::Matrix4d>> forward_kinematics(
      const std::map<std::string, std::vector<double>> &joint_values) const;

private:
  std::vector<KinematicChain> _chains;
};

} // namespace robotarm
