#include "robotarm/hand_config.hpp"

#include <fstream>
#include <stdexcept>

#include "robotarm/units.hpp"

namespace robotarm {

namespace {

double require_number(const nlohmann::json &joint_json, const std::string &field, const std::string &chain_name,
                       size_t joint_index) {
  if (!joint_json.contains(field)) {
    throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                             " is missing required field '" + field + "'");
  }
  if (!joint_json.at(field).is_number()) {
    throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                             " field '" + field + "' must be a number");
  }
  return joint_json.at(field).get<double>();
}

double optional_number(const nlohmann::json &joint_json, const std::string &field, const std::string &chain_name,
                        size_t joint_index, double default_value) {
  if (!joint_json.contains(field)) return default_value;
  if (!joint_json.at(field).is_number()) {
    throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                             " field '" + field + "' must be a number");
  }
  return joint_json.at(field).get<double>();
}

JointType parse_joint_type(const nlohmann::json &joint_json, const std::string &chain_name, size_t joint_index) {
  if (!joint_json.contains("type") || !joint_json.at("type").is_string()) {
    throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                             " is missing a string 'type' field");
  }

  const std::string type = joint_json.at("type").get<std::string>();
  if (type == "revolute") return JointType::Revolute;
  if (type == "prismatic") return JointType::Prismatic;

  throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                           " has unknown type '" + type + "' (expected 'revolute' or 'prismatic')");
}

HandJointConfig parse_joint(const nlohmann::json &joint_json, const std::string &chain_name, size_t joint_index) {
  HandJointConfig joint;
  joint.dh.type = parse_joint_type(joint_json, chain_name, joint_index);
  joint.dh.a = require_number(joint_json, "a", chain_name, joint_index);
  joint.dh.alpha = to_radians(require_number(joint_json, "alpha_deg", chain_name, joint_index));
  joint.dh.d = optional_number(joint_json, "d", chain_name, joint_index, 0.0);
  joint.dh.theta = to_radians(optional_number(joint_json, "theta_offset_deg", chain_name, joint_index, 0.0));

  joint.min_degrees = optional_number(joint_json, "min_deg", chain_name, joint_index, -180.0);
  joint.max_degrees = optional_number(joint_json, "max_deg", chain_name, joint_index, 180.0);
  if (joint.min_degrees > joint.max_degrees) {
    throw std::runtime_error("hand config: chain '" + chain_name + "' joint " + std::to_string(joint_index) +
                             " has min_deg greater than max_deg");
  }

  return joint;
}

HandChainConfig parse_chain(const nlohmann::json &chain_json, size_t chain_index) {
  if (!chain_json.contains("name") || !chain_json.at("name").is_string() ||
      chain_json.at("name").get<std::string>().empty()) {
    throw std::runtime_error("hand config: chain " + std::to_string(chain_index) +
                             " is missing a non-empty string 'name' field");
  }

  HandChainConfig chain;
  chain.name = chain_json.at("name").get<std::string>();
  chain.parent_chain = chain_json.value("parent_chain", std::string());

  if (chain_json.contains("origin")) {
    const nlohmann::json &origin = chain_json.at("origin");
    if (!origin.is_object()) {
      throw std::runtime_error("hand config: chain '" + chain.name + "' field 'origin' must be an object");
    }
    auto read_vec3 = [&](const char *field) -> Eigen::Vector3d {
      if (!origin.contains(field)) {
        return Eigen::Vector3d::Zero();
      }
      const nlohmann::json &arr = origin.at(field);
      if (!arr.is_array() || arr.size() != 3 || !arr.at(0).is_number() || !arr.at(1).is_number() ||
          !arr.at(2).is_number()) {
        throw std::runtime_error("hand config: chain '" + chain.name + "' origin." + field +
                                 " must be an array of 3 numbers");
      }
      return Eigen::Vector3d(arr.at(0).get<double>(), arr.at(1).get<double>(), arr.at(2).get<double>());
    };
    chain.origin_xyz = read_vec3("xyz");
    chain.origin_rpy_degrees = read_vec3("rpy_deg");
  }

  // A chain with no joints at all (e.g. a palm that's purely a mounting
  // point) is valid - KinematicChain allows it - so "joints" itself is
  // optional, defaulting to empty, not required.
  if (chain_json.contains("joints")) {
    if (!chain_json.at("joints").is_array()) {
      throw std::runtime_error("hand config: chain '" + chain.name + "' field 'joints' must be an array");
    }
    const nlohmann::json &joints_json = chain_json.at("joints");
    chain.joints.reserve(joints_json.size());
    for (size_t j = 0; j < joints_json.size(); j++) {
      chain.joints.push_back(parse_joint(joints_json.at(j), chain.name, j));
    }
  }

  return chain;
}

} // namespace

HandConfig parse_hand_config(const nlohmann::json &document) {
  if (!document.contains("chains") || !document.at("chains").is_array()) {
    throw std::runtime_error("hand config: missing required array field 'chains'");
  }

  const nlohmann::json &chains_json = document.at("chains");
  if (chains_json.empty()) {
    throw std::runtime_error("hand config: 'chains' must have at least one entry");
  }

  HandConfig config;
  config.name = document.value("name", std::string());
  config.chains.reserve(chains_json.size());
  for (size_t i = 0; i < chains_json.size(); i++) {
    config.chains.push_back(parse_chain(chains_json.at(i), i));
  }

  return config;
}

HandConfig load_hand_config(const std::filesystem::path &path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("hand config: could not open file '" + path.string() + "'");
  }

  nlohmann::json document;
  file >> document;
  return parse_hand_config(document);
}

KinematicTree to_kinematic_tree(const HandConfig &config) {
  std::vector<KinematicChain> chains;
  chains.reserve(config.chains.size());
  for (const HandChainConfig &chain_config : config.chains) {
    KinematicChain chain;
    chain.name = chain_config.name;
    chain.parent_chain = chain_config.parent_chain;
    const Eigen::Vector3d rpy_radians(to_radians(chain_config.origin_rpy_degrees.x()),
                                      to_radians(chain_config.origin_rpy_degrees.y()),
                                      to_radians(chain_config.origin_rpy_degrees.z()));
    chain.origin = xyz_rpy_transform(chain_config.origin_xyz, rpy_radians);
    chain.joints.reserve(chain_config.joints.size());
    for (const HandJointConfig &joint : chain_config.joints) chain.joints.push_back(joint.dh);
    chains.push_back(std::move(chain));
  }
  return KinematicTree(std::move(chains));
}

} // namespace robotarm
