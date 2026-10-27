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

double optional_number(const nlohmann::json &joint_json, const std::string &field, size_t index, double default_value) {
  if (!joint_json.contains(field)) return default_value;
  if (!joint_json.at(field).is_number()) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " field '" + field + "' must be a number");
  }
  return joint_json.at(field).get<double>();
}

Eigen::Vector3d parse_vector3(const nlohmann::json &joint_json, const std::string &field, size_t index) {
  const nlohmann::json &array = joint_json.at(field);
  if (!array.is_array() || array.size() != 3) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " field '" + field +
                             "' must be an array of 3 numbers");
  }
  return Eigen::Vector3d(array.at(0).get<double>(), array.at(1).get<double>(), array.at(2).get<double>());
}

// [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (kg*m^2), the independent components of
// a symmetric 3x3 inertia tensor.
Eigen::Matrix3d parse_inertia6(const nlohmann::json &joint_json, const std::string &field, size_t index) {
  const nlohmann::json &array = joint_json.at(field);
  if (!array.is_array() || array.size() != 6) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " field '" + field +
                             "' must be an array of 6 numbers [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]");
  }
  const double ixx = array.at(0).get<double>();
  const double iyy = array.at(1).get<double>();
  const double izz = array.at(2).get<double>();
  const double ixy = array.at(3).get<double>();
  const double ixz = array.at(4).get<double>();
  const double iyz = array.at(5).get<double>();

  Eigen::Matrix3d inertia;
  inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  return inertia;
}

JointType parse_joint_type(const nlohmann::json &joint_json, size_t index) {
  if (!joint_json.contains("type") || !joint_json.at("type").is_string()) {
    throw std::runtime_error("arm config: joint " + std::to_string(index) + " is missing a string 'type' field");
  }

  const std::string type = joint_json.at("type").get<std::string>();

  if (type == "revolute") return JointType::Revolute;
  if (type == "prismatic") return JointType::Prismatic;

  throw std::runtime_error("arm config: joint " + std::to_string(index) + " has unknown type '" + type + "' (expected 'revolute' or 'prismatic')");
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

    const bool has_com = joint_json.contains("center_of_mass");
    const bool has_inertia = joint_json.contains("inertia");
    if (has_com != has_inertia) {
      throw std::runtime_error("arm config: joint " + std::to_string(i) +
                               " has 'center_of_mass' without 'inertia' (or vice versa) - give both or neither");
    }
    if (has_com) {
      joint.inertial =
          LinkInertial(joint.mass, parse_vector3(joint_json, "center_of_mass", i), parse_inertia6(joint_json, "inertia", i));
    } else {
      // Documented default: a uniform slender rod along this joint's own
      // link - see LinkInertial::rod()'s own doc comment for when this
      // placement is (and isn't) correct.
      joint.inertial = LinkInertial::rod(joint.mass, joint.dh.a);
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

std::vector<LinkInertial> to_link_inertials(const ArmConfig &config) {
  std::vector<LinkInertial> links;
  links.reserve(config.joints.size());
  for (const JointConfig &joint : config.joints) {
    links.push_back(joint.inertial);
  }
  return links;
}

} // namespace robotarm
