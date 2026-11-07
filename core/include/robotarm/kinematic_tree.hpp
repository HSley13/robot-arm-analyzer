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
//
// `origin` is a fixed transform applied between the parent's tip (or the
// world origin) and this chain's first DH joint - the same idea as a URDF
// joint's <origin xyz rpy>. Pure DH can only place a branch by rotating
// about the previous z axis and sliding along x, which can't express e.g.
// a thumb whose base sits off to the side of the palm and points
// forward-and-sideways at once. Identity by default, so a chain that
// doesn't need it behaves exactly as before.
struct KinematicChain {
  std::string name;
  std::string parent_chain; // empty => attaches to the world origin
  std::vector<DHParameter> joints;
  Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
};

// Builds a homogeneous transform from a translation and URDF-convention
// roll/pitch/yaw (radians, fixed axes: R = Rz(yaw) * Ry(pitch) * Rx(roll)).
Eigen::Matrix4d xyz_rpy_transform(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy_radians);

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

  // The world-frame pose of one chain's own base - its parent's tip (or
  // the world origin) times its fixed `origin` - i.e. the frame its first
  // joint rotates in. Not part of forward_kinematics()'s per-joint list
  // (which has exactly one entry per joint), so it's exposed separately;
  // `poses` is a forward_kinematics() result for the same tree.
  Eigen::Matrix4d chain_base_frame(const std::string &name,
                                   const std::map<std::string, std::vector<Eigen::Matrix4d>> &poses) const;

private:
  std::vector<KinematicChain> _chains;
};

} // namespace robotarm
