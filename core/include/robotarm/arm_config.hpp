#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robotarm/dh.hpp"

namespace robotarm {

// One joint's full definition as it appears in an arm config file: its DH
// geometry plus the physical properties the statics/UI layers need. Angles
// in the JSON are authored in DEGREES (explicit in the field names:
// alpha_deg, theta_offset_deg) and converted to radians at this exact
// boundary - see units.hpp. torque_limit is N*m for a Revolute joint, N
// (a force limit) for a Prismatic one - the same field reused as a generic
// "effort limit" to keep the schema simple.
struct JointConfig {
  DHParameter dh;
  double mass = 0.0;         // kg, at this joint's frame origin - see statics.hpp
  double torque_limit = 0.0; // N*m (Revolute) or N (Prismatic)
};

// A full serial-link arm definition, loaded from JSON so the tool works for
// any arm instead of one hardcoded one (the project's core generality goal).
struct ArmConfig {
  std::string name;
  std::vector<JointConfig> joints;
};

// Parses an already-loaded JSON document into an ArmConfig. Throws
// std::runtime_error with a specific message (which joint, what's wrong) on
// any missing or malformed field - fail loudly on a bad config, don't
// guess a default for something safety-relevant like a torque limit.
ArmConfig parse_arm_config(const nlohmann::json &document);

} // namespace robotarm
