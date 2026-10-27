#include "robotarm/arm_config.hpp"

#include <fstream>
#include <stdexcept>

#include "robotarm/units.hpp"

namespace robotarm {

namespace {

double require_number(const nlohmann::json &joint_json, const std::string &field, size_t index) {
  if (!joint_json.contains(field)) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " is missing required field '" +
                              field + "'");
  }
  if (!joint_json.at(field).is_number()) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " field '" + field +
                              "' must be a number");
  }
  return joint_json.at(field).get<double>();
}

double optional_number(const nlohmann::json &joint_json, const std::string &field, size_t index,
                        double default_value) {
  if (!joint_json.contains(field)) return default_value;
  if (!joint_json.at(field).is_number()) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " field '" + field +
                              "' must be a number");
  }
  return joint_json.at(field).get<double>();
}

JointType parse_joint_type(const nlohmann::json &joint_json, size_t index) {
  if (!joint_json.contains("type") || !joint_json.at("type").is_string()) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " is missing a string 'type' field");
  }
  const std::string type = joint_json.at("type").get<std::string>();
  if (type == "revolute") return JointType::Revolute;
  if (type == "prismatic") return JointType::Prismatic;
  throw std::runtime_error("arm config: joint " + std::to_string(index) + " has unknown type '" + type +
                            "' (expected 'revolute' or 'prismatic')");
}

} // namespace

ArmConfig parse_arm_config(const nlohmann::json &document) {
  if (!document.contains("joints") || !document.at("joints").is_array()) {
    throw std::runtime_error("arm config: missing required array field 'joints'");
  }
  const nlohmann::json &joints_json = document.at("joints");
  if (joints_json.empty()) {
    throw std::runtime_error("arm config: 'joints' must have at least one entry");
  }

  ArmConfig config;
  config.name = document.value("name", std::string());

  for (size_t i = 0; i < joints_json.size(); i++) {
    const nlohmann::json &joint_json = joints_json.at(i);

    JointConfig joint;
    joint.dh.type = parse_joint_type(joint_json, i);
    joint.dh.a = require_number(joint_json, "a", i);
    joint.dh.alpha = to_radians(require_number(joint_json, "alpha_deg", i));
    joint.dh.d = optional_number(joint_json, "d", i, 0.0);
    joint.dh.theta = to_radians(optional_number(joint_json, "theta_offset_deg", i, 0.0));

    joint.mass = optional_number(joint_json, "mass", i, 0.0);
    if (joint.mass < 0.0) {
      throw std::runtime_error("arm config: joint " + std::to_string(i) + " has negative mass");
    }

    joint.torque_limit = require_number(joint_json, "torque_limit", i);
    if (joint.torque_limit <= 0.0) {
      throw std::runtime_error("arm config: joint " + std::to_string(i) + " must have a positive torque_limit");
    }

    config.joints.push_back(joint);
  }

  return config;
}

ArmConfig load_arm_config(const std::filesystem::path &path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("arm config: could not open file '" + path.string() + "'");
  }

  nlohmann::json document;
  file >> document;
  return parse_arm_config(document);
}

ArmModel to_arm_model(const ArmConfig &config) {
  std::vector<DHParameter> joints;
  joints.reserve(config.joints.size());
  for (const JointConfig &joint : config.joints) {
    joints.push_back(joint.dh);
  }
  return ArmModel(joints);
}

std::vector<LinkMass> to_link_masses(const ArmConfig &config) {
  std::vector<LinkMass> links;
  links.reserve(config.joints.size());
  for (const JointConfig &joint : config.joints) {
    links.push_back(LinkMass(joint.mass));
  }
  return links;
}

} // namespace robotarm
