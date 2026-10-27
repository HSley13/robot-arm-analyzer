// Verifies KinematicTree: a single chain with no parent must match
// ArmModel exactly (it's the same math, just addressed by name), two
// independent branches sharing one parent must not cross-contaminate each
// other, a chain inherits its parent's actual tip pose (not just
// identity) when the parent itself has joints, and malformed trees
// (duplicate names, an unresolvable parent, zero chains) fail loudly.
#include <stdexcept>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/kinematic_tree.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::KinematicChain;
using robotarm::KinematicTree;

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

TEST(KinematicTree, SingleRootChainMatchesArmModelExactly) {
  KinematicChain chain;
  chain.name = "arm";
  chain.joints = {DHParameter(1.0), DHParameter(0.5)};

  const KinematicTree tree({chain});
  const double theta1 = 30.0 * kPi / 180.0;
  const double theta2 = 45.0 * kPi / 180.0;

  const auto result = tree.forward_kinematics({{"arm", {theta1, theta2}}});
  const std::vector<Eigen::Matrix4d> &tree_poses = result.at("arm");

  const ArmModel arm_model({DHParameter(1.0), DHParameter(0.5)});
  const std::vector<Eigen::Matrix4d> model_poses = arm_model.forward_kinematics({theta1, theta2});

  ASSERT_EQ(tree_poses.size(), model_poses.size());
  for (size_t i = 0; i < tree_poses.size(); i++) {
    EXPECT_TRUE(tree_poses[i].isApprox(model_poses[i], 1e-9)) << "joint " << i;
  }
}

TEST(KinematicTree, IndependentBranchesDoNotCrossContaminate) {
  // A palm (no joints of its own - a fixed mounting point) with two
  // fingers attached to it, each a different length and pointed a
  // different way - the genuinely different topology from a single
  // chain.
  KinematicChain palm;
  palm.name = "palm";

  KinematicChain finger1;
  finger1.name = "finger1";
  finger1.parent_chain = "palm";
  finger1.joints = {DHParameter(0.3)};

  KinematicChain finger2;
  finger2.name = "finger2";
  finger2.parent_chain = "palm";
  finger2.joints = {DHParameter(0.3, kPi / 2.0)}; // alpha=90deg: a different axis than finger1

  const KinematicTree tree({palm, finger1, finger2});

  const auto result_a = tree.forward_kinematics({{"palm", {}}, {"finger1", {0.2}}, {"finger2", {0.2}}});
  const auto result_b = tree.forward_kinematics({{"palm", {}}, {"finger1", {1.7}}, {"finger2", {0.2}}});

  // Moving finger1 alone must not change finger2's computed pose.
  ASSERT_EQ(result_a.at("finger2").size(), result_b.at("finger2").size());
  for (size_t i = 0; i < result_a.at("finger2").size(); i++) {
    EXPECT_TRUE(result_a.at("finger2")[i].isApprox(result_b.at("finger2")[i], 1e-9)) << "finger2 joint " << i;
  }
  // But finger1 itself must actually have moved (otherwise this test
  // would pass trivially for the wrong reason).
  EXPECT_FALSE(result_a.at("finger1").back().isApprox(result_b.at("finger1").back(), 1e-6));
}

TEST(KinematicTree, ChildChainInheritsParentsActualTipPose) {
  // A wrist with one joint, then a finger attached to the wrist's tip -
  // moving the wrist must move the finger's whole base frame with it,
  // not just leave it at identity.
  KinematicChain wrist;
  wrist.name = "wrist";
  wrist.joints = {DHParameter(0.0, kPi / 2.0)}; // pure twist, no length - a mounting reorientation

  KinematicChain finger;
  finger.name = "finger";
  finger.parent_chain = "wrist";
  finger.joints = {DHParameter(0.4)};

  const KinematicTree tree({wrist, finger});

  const auto result_rest = tree.forward_kinematics({{"wrist", {0.0}}, {"finger", {0.0}}});
  const auto result_rotated = tree.forward_kinematics({{"wrist", {kPi / 2.0}}, {"finger", {0.0}}});

  // Same finger joint value (0.0) in both cases, but the wrist rotated by
  // 90deg in between - the finger's resulting world pose must differ,
  // proving it actually inherited the wrist's new tip transform.
  EXPECT_FALSE(result_rest.at("finger").back().isApprox(result_rotated.at("finger").back(), 1e-6));
}

TEST(KinematicTree, EmptyChainListThrows) { EXPECT_THROW(KinematicTree({}), std::invalid_argument); }

TEST(KinematicTree, DuplicateChainNameThrows) {
  KinematicChain a;
  a.name = "x";
  a.joints = {DHParameter(1.0)};
  KinematicChain b;
  b.name = "x";
  b.joints = {DHParameter(1.0)};
  EXPECT_THROW(KinematicTree({a, b}), std::invalid_argument);
}

TEST(KinematicTree, UnresolvableParentThrows) {
  KinematicChain orphan;
  orphan.name = "finger";
  orphan.parent_chain = "nonexistent_palm";
  orphan.joints = {DHParameter(1.0)};
  EXPECT_THROW(KinematicTree({orphan}), std::invalid_argument);
}

TEST(KinematicTree, ChainAccessorFindsByNameAndThrowsWhenMissing) {
  KinematicChain a;
  a.name = "x";
  a.joints = {DHParameter(1.0)};
  const KinematicTree tree({a});

  EXPECT_EQ(tree.chain("x").name, "x");
  EXPECT_THROW(tree.chain("missing"), std::out_of_range);
}
