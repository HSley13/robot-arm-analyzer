// Verifies HandConfig: parsing a hand-config JSON document produces the
// right chain/joint structure with correct deg->rad conversion at the
// boundary, malformed documents fail loudly (missing fields, empty
// 'chains', a joint with min_deg > max_deg), a chain with zero joints
// (a pure mounting point) is valid, and to_kinematic_tree() both builds a
// working KinematicTree and still fails loudly on bad topology (an
// unresolvable parent_chain) via KinematicTree's own constructor. The
// last test loads the actual shipped config/humanoid_hand.json, so a
// mistake in that hand-written data file - not just in the parser - gets
// caught too.
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "robotarm/hand_config.hpp"
#include "robotarm/units.hpp"

using robotarm::HandConfig;
using robotarm::JointType;
using robotarm::KinematicTree;
using robotarm::load_hand_config;
using robotarm::parse_hand_config;
using robotarm::to_kinematic_tree;
using robotarm::to_radians;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string example_config_path() { return std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"; }

nlohmann::json minimal_two_chain_document() {
  return nlohmann::json::parse(R"({
    "name": "Test Hand",
    "chains": [
      {
        "name": "wrist",
        "parent_chain": "",
        "joints": [
          { "type": "revolute", "a": 0.0, "alpha_deg": 90.0, "d": 0.0, "theta_offset_deg": 0.0 }
        ]
      },
      {
        "name": "index",
        "parent_chain": "wrist",
        "joints": [
          { "type": "revolute", "a": 0.04, "alpha_deg": 0.0, "min_deg": -15.0, "max_deg": 90.0 }
        ]
      }
    ]
  })");
}

} // namespace

TEST(HandConfig, ParsesChainsAndJointsWithDegreesConvertedToRadians) {
  const HandConfig config = parse_hand_config(minimal_two_chain_document());

  EXPECT_EQ(config.name, "Test Hand");
  ASSERT_EQ(config.chains.size(), 2u);

  EXPECT_EQ(config.chains[0].name, "wrist");
  EXPECT_EQ(config.chains[0].parent_chain, "");
  ASSERT_EQ(config.chains[0].joints.size(), 1u);
  EXPECT_NEAR(config.chains[0].joints[0].dh.alpha, to_radians(90.0), 1e-12);
  // min/max default to the full range when omitted.
  EXPECT_DOUBLE_EQ(config.chains[0].joints[0].min_degrees, -180.0);
  EXPECT_DOUBLE_EQ(config.chains[0].joints[0].max_degrees, 180.0);

  EXPECT_EQ(config.chains[1].name, "index");
  EXPECT_EQ(config.chains[1].parent_chain, "wrist");
  ASSERT_EQ(config.chains[1].joints.size(), 1u);
  EXPECT_DOUBLE_EQ(config.chains[1].joints[0].dh.a, 0.04);
  EXPECT_EQ(config.chains[1].joints[0].dh.type, JointType::Revolute);
  EXPECT_DOUBLE_EQ(config.chains[1].joints[0].min_degrees, -15.0);
  EXPECT_DOUBLE_EQ(config.chains[1].joints[0].max_degrees, 90.0);
}

TEST(HandConfig, ChainWithNoJointsIsValid) {
  const nlohmann::json document = nlohmann::json::parse(R"({
    "chains": [ { "name": "palm", "parent_chain": "" } ]
  })");

  const HandConfig config = parse_hand_config(document);
  ASSERT_EQ(config.chains.size(), 1u);
  EXPECT_EQ(config.chains[0].name, "palm");
  EXPECT_TRUE(config.chains[0].joints.empty());

  // And it must still build a valid (trivial) KinematicTree.
  const KinematicTree tree = to_kinematic_tree(config);
  EXPECT_EQ(tree.num_chains(), 1u);
}

TEST(HandConfig, MissingChainsFieldThrows) {
  EXPECT_THROW(parse_hand_config(nlohmann::json::parse(R"({"name": "x"})")), std::runtime_error);
}

TEST(HandConfig, EmptyChainsArrayThrows) {
  EXPECT_THROW(parse_hand_config(nlohmann::json::parse(R"({"chains": []})")), std::runtime_error);
}

TEST(HandConfig, ChainMissingNameThrows) {
  const nlohmann::json document = nlohmann::json::parse(R"({"chains": [ {"parent_chain": ""} ]})");
  EXPECT_THROW(parse_hand_config(document), std::runtime_error);
}

TEST(HandConfig, JointMissingRequiredFieldThrows) {
  // Missing "a".
  const nlohmann::json document = nlohmann::json::parse(R"({
    "chains": [ { "name": "wrist", "joints": [ {"type": "revolute", "alpha_deg": 0.0} ] } ]
  })");
  EXPECT_THROW(parse_hand_config(document), std::runtime_error);
}

TEST(HandConfig, JointWithMinGreaterThanMaxThrows) {
  const nlohmann::json document = nlohmann::json::parse(R"({
    "chains": [ { "name": "wrist", "joints": [
      {"type": "revolute", "a": 0.0, "alpha_deg": 0.0, "min_deg": 50.0, "max_deg": -50.0}
    ] } ]
  })");
  EXPECT_THROW(parse_hand_config(document), std::runtime_error);
}

TEST(HandConfig, MinEqualsMaxIsAllowedAsAFixedPlacementJoint) {
  const nlohmann::json document = nlohmann::json::parse(R"({
    "chains": [ { "name": "wrist", "joints": [
      {"type": "revolute", "a": 0.04, "alpha_deg": 0.0, "min_deg": 0.0, "max_deg": 0.0}
    ] } ]
  })");
  const HandConfig config = parse_hand_config(document);
  EXPECT_DOUBLE_EQ(config.chains[0].joints[0].min_degrees, config.chains[0].joints[0].max_degrees);
}

TEST(HandConfig, ToKinematicTreeMatchesManuallyBuiltTree) {
  const HandConfig config = parse_hand_config(minimal_two_chain_document());
  const KinematicTree tree = to_kinematic_tree(config);

  ASSERT_EQ(tree.num_chains(), 2u);
  const auto result = tree.forward_kinematics({{"wrist", {0.3}}, {"index", {0.2}}});
  ASSERT_EQ(result.at("wrist").size(), 1u);
  ASSERT_EQ(result.at("index").size(), 1u);
  // The index chain inherited the wrist's tip, not identity - its pose
  // must differ from what it would be if "wrist" were left at 0.
  const auto result_wrist_at_rest = tree.forward_kinematics({{"wrist", {0.0}}, {"index", {0.2}}});
  EXPECT_FALSE(result.at("index")[0].isApprox(result_wrist_at_rest.at("index")[0], 1e-9));
}

TEST(HandConfig, ToKinematicTreeThrowsOnUnresolvableParent) {
  const nlohmann::json document = nlohmann::json::parse(R"({
    "chains": [ { "name": "index", "parent_chain": "nonexistent_wrist", "joints": [
      {"type": "revolute", "a": 0.04, "alpha_deg": 0.0}
    ] } ]
  })");
  const HandConfig config = parse_hand_config(document);
  EXPECT_THROW(to_kinematic_tree(config), std::invalid_argument);
}

TEST(HandConfig, LoadHandConfigMissingFileThrows) {
  EXPECT_THROW(load_hand_config("/nonexistent/hand.json"), std::runtime_error);
}

TEST(HandConfig, LoadsTheShippedExampleConfigAndBuildsAValidTree) {
  const HandConfig config = load_hand_config(example_config_path());

  // wrist + index + middle + ring + pinky + thumb.
  ASSERT_EQ(config.chains.size(), 6u);

  const KinematicTree tree = to_kinematic_tree(config);
  EXPECT_EQ(tree.num_chains(), 6u);

  std::map<std::string, std::vector<double>> rest_pose;
  for (const auto &chain : config.chains) rest_pose[chain.name] = std::vector<double>(chain.joints.size(), 0.0);

  const auto result = tree.forward_kinematics(rest_pose);
  EXPECT_EQ(result.size(), 6u);
  for (const auto &chain : config.chains) {
    ASSERT_EQ(result.at(chain.name).size(), chain.joints.size()) << "chain '" << chain.name << "'";
  }
}
