// Verifies the JSON arm config loader: parsing a valid config, converting
// degrees to radians at the exact boundary, converting to ArmModel/LinkMass
// and getting physics identical to a hand-built arm (the whole point of a
// config loader is that it must not silently change the numbers), loading
// an actual file from disk, and failing loudly - not silently guessing -
// on malformed input.
#include <filesystem>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "robotarm/arm_config.hpp"
#include "robotarm/statics.hpp"

using robotarm::ArmConfig;
using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::LinkInertial;
using robotarm::LinkMass;
using robotarm::load_arm_config;
using robotarm::parse_arm_config;
using robotarm::save_arm_config;
using robotarm::static_joint_torques;
using robotarm::to_arm_model;
using robotarm::to_json;
using robotarm::to_link_inertials;
using robotarm::to_link_masses;

namespace {

constexpr double kPi = 3.14159265358979323846;

nlohmann::json two_link_planar_json() {
  return nlohmann::json::parse(R"json({
    "name": "Two-Link Planar Arm (test)",
    "joints": [
      {"type": "revolute", "a": 1.0, "alpha_deg": 0.0, "d": 0.0, "theta_offset_deg": 0.0, "mass": 2.0, "torque_limit": 50.0},
      {"type": "revolute", "a": 0.5, "alpha_deg": 0.0, "d": 0.0, "theta_offset_deg": 0.0, "mass": 1.0, "torque_limit": 25.0}
    ]
  })json");
}

} // namespace

TEST(ArmConfig, ParsesValidConfig) {
  const ArmConfig config = parse_arm_config(two_link_planar_json());

  EXPECT_EQ(config.name, "Two-Link Planar Arm (test)");
  ASSERT_EQ(config.joints.size(), 2u);

  EXPECT_DOUBLE_EQ(config.joints[0].dh.a, 1.0);
  EXPECT_NEAR(config.joints[0].dh.alpha, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(config.joints[0].mass, 2.0);
  EXPECT_DOUBLE_EQ(config.joints[0].torque_limit, 50.0);

  EXPECT_DOUBLE_EQ(config.joints[1].dh.a, 0.5);
  EXPECT_DOUBLE_EQ(config.joints[1].mass, 1.0);
}

TEST(ArmConfig, DegreesFieldsConvertToRadiansAtTheBoundary) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["alpha_deg"] = 90.0;
  doc["joints"][0]["theta_offset_deg"] = 180.0;

  const ArmConfig config = parse_arm_config(doc);
  EXPECT_NEAR(config.joints[0].dh.alpha, kPi / 2.0, 1e-12);
  EXPECT_NEAR(config.joints[0].dh.theta, kPi, 1e-12);
}

TEST(ArmConfig, ConversionMatchesHandBuiltArmExactly) {
  const ArmConfig config = parse_arm_config(two_link_planar_json());
  const ArmModel arm = to_arm_model(config);
  const std::vector<LinkMass> links = to_link_masses(config);

  const std::vector<double> theta = {30.0 * kPi / 180.0, 45.0 * kPi / 180.0};
  const double payload = 0.5;
  const Eigen::Vector3d gravity_down_y(0.0, -9.81, 0.0);

  const Eigen::VectorXd tau_from_config = static_joint_torques(arm, theta, links, payload, gravity_down_y);

  const ArmModel hand_built_arm({DHParameter(1.0), DHParameter(0.5)});
  const std::vector<LinkMass> hand_built_links = {LinkMass(2.0), LinkMass(1.0)};
  const Eigen::VectorXd tau_hand_built =
      static_joint_torques(hand_built_arm, theta, hand_built_links, payload, gravity_down_y);

  ASSERT_EQ(tau_from_config.size(), tau_hand_built.size());
  EXPECT_NEAR(tau_from_config(0), tau_hand_built(0), 1e-12);
  EXPECT_NEAR(tau_from_config(1), tau_hand_built(1), 1e-12);
}

TEST(ArmConfig, LoadsFromFile) {
#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif
  const ArmConfig config = load_arm_config(std::string(PROJECT_ROOT_DIR) + "/config/two_link_planar.json");
  ASSERT_EQ(config.joints.size(), 2u);
  EXPECT_DOUBLE_EQ(config.joints[0].dh.a, 1.0);
  EXPECT_DOUBLE_EQ(config.joints[1].torque_limit, 25.0);
}

TEST(ArmConfig, LoadsThreeDofConfigWithPrismaticJoint) {
  const ArmConfig config =
      load_arm_config(std::string(PROJECT_ROOT_DIR) + "/config/three_dof_with_prismatic.json");
  ASSERT_EQ(config.joints.size(), 3u);
  EXPECT_NEAR(config.joints[0].dh.alpha, -kPi / 2.0, 1e-12);
  EXPECT_EQ(config.joints[2].dh.type, robotarm::JointType::Prismatic);
}

TEST(ArmConfig, MissingTorqueLimitThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0].erase("torque_limit");
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, UnknownJointTypeThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["type"] = "rotary";
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, EmptyJointsArrayThrows) {
  nlohmann::json doc;
  doc["joints"] = nlohmann::json::array();
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, NegativeMassThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["mass"] = -1.0;
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, NonPositiveTorqueLimitThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["torque_limit"] = 0.0;
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, LoadingMissingFileThrows) {
  EXPECT_THROW(load_arm_config("/nonexistent/path/does_not_exist.json"), std::runtime_error);
}

TEST(ArmConfig, DefaultsToRodInertialWhenOmitted) {
  const ArmConfig config = parse_arm_config(two_link_planar_json());
  const std::vector<LinkInertial> links = to_link_inertials(config);

  ASSERT_EQ(links.size(), 2u);
  const LinkInertial expected_link0 = LinkInertial::rod(2.0, 1.0);
  EXPECT_DOUBLE_EQ(links[0].mass, expected_link0.mass);
  EXPECT_NEAR(links[0].center_of_mass.x(), expected_link0.center_of_mass.x(), 1e-12);
  EXPECT_NEAR(links[0].inertia(1, 1), expected_link0.inertia(1, 1), 1e-12);
}

TEST(ArmConfig, UsesExplicitInertialDataWhenGiven) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["center_of_mass"] = {-0.3, 0.1, 0.0};
  doc["joints"][0]["inertia"] = {0.01, 0.02, 0.03, 0.0, 0.0, 0.0};

  const ArmConfig config = parse_arm_config(doc);
  const std::vector<LinkInertial> links = to_link_inertials(config);

  EXPECT_NEAR(links[0].center_of_mass.x(), -0.3, 1e-12);
  EXPECT_NEAR(links[0].center_of_mass.y(), 0.1, 1e-12);
  EXPECT_NEAR(links[0].inertia(0, 0), 0.01, 1e-12);
  EXPECT_NEAR(links[0].inertia(1, 1), 0.02, 1e-12);
  EXPECT_NEAR(links[0].inertia(2, 2), 0.03, 1e-12);
}

TEST(ArmConfig, CenterOfMassWithoutInertiaThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["center_of_mass"] = {-0.3, 0.0, 0.0};
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, InertiaWithoutCenterOfMassThrows) {
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["inertia"] = {0.01, 0.02, 0.03, 0.0, 0.0, 0.0};
  EXPECT_THROW(parse_arm_config(doc), std::runtime_error);
}

TEST(ArmConfig, RoundTripThroughJsonReproducesTheOriginalConfigExactly) {
  // Use a config with non-default inertial data and non-zero d/theta_offset
  // so the round trip actually exercises every field, not just the ones
  // that happen to equal their defaults.
  nlohmann::json doc = two_link_planar_json();
  doc["joints"][0]["d"] = 0.2;
  doc["joints"][0]["theta_offset_deg"] = 15.0;
  doc["joints"][0]["center_of_mass"] = {-0.3, 0.1, 0.05};
  doc["joints"][0]["inertia"] = {0.01, 0.02, 0.03, 0.001, 0.002, 0.003};

  const ArmConfig original = parse_arm_config(doc);
  const ArmConfig reloaded = parse_arm_config(to_json(original));

  EXPECT_EQ(reloaded.name, original.name);
  ASSERT_EQ(reloaded.joints.size(), original.joints.size());

  for (size_t i = 0; i < original.joints.size(); i++) {
    const auto &a = original.joints[i];
    const auto &b = reloaded.joints[i];

    EXPECT_EQ(b.dh.type, a.dh.type) << "joint " << i;
    EXPECT_DOUBLE_EQ(b.dh.a, a.dh.a) << "joint " << i;
    EXPECT_NEAR(b.dh.alpha, a.dh.alpha, 1e-12) << "joint " << i;
    EXPECT_DOUBLE_EQ(b.dh.d, a.dh.d) << "joint " << i;
    EXPECT_NEAR(b.dh.theta, a.dh.theta, 1e-12) << "joint " << i;
    EXPECT_DOUBLE_EQ(b.mass, a.mass) << "joint " << i;
    EXPECT_DOUBLE_EQ(b.torque_limit, a.torque_limit) << "joint " << i;

    EXPECT_NEAR(b.inertial.center_of_mass.x(), a.inertial.center_of_mass.x(), 1e-12) << "joint " << i;
    EXPECT_NEAR(b.inertial.center_of_mass.y(), a.inertial.center_of_mass.y(), 1e-12) << "joint " << i;
    EXPECT_NEAR(b.inertial.center_of_mass.z(), a.inertial.center_of_mass.z(), 1e-12) << "joint " << i;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        EXPECT_NEAR(b.inertial.inertia(row, col), a.inertial.inertia(row, col), 1e-12)
            << "joint " << i << " inertia(" << row << "," << col << ")";
      }
    }
  }
}

TEST(ArmConfig, SaveThenLoadRoundTripsThroughAnActualFile) {
  const ArmConfig original = parse_arm_config(two_link_planar_json());
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "robotarm_arm_config_roundtrip_test.json";

  save_arm_config(original, path);
  const ArmConfig reloaded = load_arm_config(path);
  std::filesystem::remove(path);

  EXPECT_EQ(reloaded.name, original.name);
  ASSERT_EQ(reloaded.joints.size(), original.joints.size());
  EXPECT_DOUBLE_EQ(reloaded.joints[0].dh.a, original.joints[0].dh.a);
  EXPECT_DOUBLE_EQ(reloaded.joints[1].torque_limit, original.joints[1].torque_limit);
}

TEST(ArmConfig, SavingToAnUnwritablePathThrows) {
  const ArmConfig config = parse_arm_config(two_link_planar_json());
  EXPECT_THROW(save_arm_config(config, "/nonexistent/directory/does_not_exist.json"), std::runtime_error);
}
