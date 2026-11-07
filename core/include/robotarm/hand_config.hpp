#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "robotarm/dh.hpp"
#include "robotarm/kinematic_tree.hpp"

namespace robotarm {

// One joint's full definition as it appears in a hand config file: its DH
// geometry plus the angle range it's mechanically allowed to move through.
// min_degrees/max_degrees are intentionally kept in degrees, not converted
// to radians like dh.alpha/dh.theta are - they're a UI/gesture-clamping
// concern read directly at the boundary, not a kinematics quantity.
//
// Mass/torque fields are deliberately NOT part of this schema yet - the
// hand's first milestone is pure kinematics (posing, gestures), not load
// analysis. Those belong here once grasp-force estimation is actually
// designed, the same way ArmConfig's torque_limit exists for a purpose
// this stage doesn't have.
//
// A joint with min_degrees == max_degrees (typically both 0) is a fixed
// placement, not a real gesture target - used for a finger's own
// metacarpal offset from the wrist's tip (every real finger has one, it's
// just normally fused into the palm rather than separately actuated).
// KinematicChain has no separate notion of "fixed" vs "actuated" joints,
// so this is the schema's way of marking one without inventing a new
// concept: gesture/control code should simply never target a zero-range
// joint.
struct HandJointConfig {
  DHParameter dh;
  double min_degrees = -180.0;
  double max_degrees = 180.0;
};

// One branch of the hand - a finger, or the wrist/forearm chain the
// fingers attach to - as it appears in a hand config file. Mirrors
// KinematicChain's own shape (name + parent_chain + joints) but with the
// richer per-joint HandJointConfig instead of a bare DHParameter.
//
// `origin_xyz`/`origin_rpy_degrees` are the optional fixed placement of
// the chain's base relative to its parent's tip (JSON: "origin": {"xyz":
// [x, y, z], "rpy_deg": [roll, pitch, yaw]}) - see KinematicChain::origin.
// Both default to zero, i.e. the chain starts exactly at its parent's tip.
struct HandChainConfig {
  std::string name;
  std::string parent_chain; // empty => this chain's own local root
  std::vector<HandJointConfig> joints;
  Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_rpy_degrees = Eigen::Vector3d::Zero();
};

// A full hand definition, loaded from JSON: one wrist/forearm chain plus
// however many finger chains are parented to it. Nothing here assumes
// exactly five fingers or exactly four joints per finger - a different
// chain list is just a different hand, the same generality goal
// ArmConfig/KinematicTree already have.
struct HandConfig {
  std::string name;
  std::vector<HandChainConfig> chains;
};

// Parses an already-loaded JSON document into a HandConfig. Throws
// std::runtime_error with a specific message (which chain, which joint,
// what's wrong) on any missing or malformed field. Does NOT validate tree
// topology (duplicate chain names, an unresolvable parent) - that's
// to_kinematic_tree()'s job, via KinematicTree's own constructor, the
// same deferred-validation split ArmConfig/to_arm_model() already use.
HandConfig parse_hand_config(const nlohmann::json &document);

// Reads and parses a hand config file from disk. Throws std::runtime_error
// if the file can't be opened, in addition to parse_hand_config()'s own
// validation errors.
HandConfig load_hand_config(const std::filesystem::path &path);

// Converts a HandConfig into the KinematicTree the forward-kinematics
// layer operates on - drops the angle limits, which are a UI/gesture-
// clamping concern, not a kinematics one. Throws std::invalid_argument
// (via KinematicTree's constructor) if the chains don't form a valid tree.
KinematicTree to_kinematic_tree(const HandConfig &config);

} // namespace robotarm
