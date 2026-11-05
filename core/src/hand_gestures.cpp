#include "robotarm/hand_gestures.hpp"

#include "robotarm/units.hpp"

namespace robotarm {

const HandChainConfig *HandGestures::findChain(const std::string &name) const {
  for (const HandChainConfig &chain : _config.chains) {
    if (chain.name == name) return &chain;
  }
  return nullptr;
}

const HandChainConfig *HandGestures::rootChain() const {
  for (const HandChainConfig &chain : _config.chains) {
    if (chain.parent_chain.empty()) return &chain;
  }
  return nullptr;
}

std::map<std::string, std::vector<double>> HandGestures::restPose() const {
  std::map<std::string, std::vector<double>> pose;
  for (const HandChainConfig &chain : _config.chains) {
    pose[chain.name] = std::vector<double>(chain.joints.size(), 0.0);
  }
  return pose;
}

std::map<std::string, std::vector<double>> HandGestures::poseAtFraction(double fraction, bool includeRootChain) const {
  std::map<std::string, std::vector<double>> pose;
  for (const HandChainConfig &chain : _config.chains) {
    std::vector<double> angles(chain.joints.size(), 0.0);
    const bool is_root_chain = chain.parent_chain.empty();
    if (!is_root_chain || includeRootChain) {
      for (size_t j = 0; j < chain.joints.size(); j++) {
        const HandJointConfig &joint = chain.joints[j];
        if (joint.min_degrees == joint.max_degrees) continue; // fixed placement - never moves
        const double degrees = joint.min_degrees + fraction * (joint.max_degrees - joint.min_degrees);
        angles[j] = to_radians(degrees);
      }
    }
    pose[chain.name] = angles;
  }
  return pose;
}

std::map<std::string, std::vector<double>> HandGestures::openPalm() const {
  return poseAtFraction(0.0, /*includeRootChain=*/false);
}

std::map<std::string, std::vector<double>> HandGestures::closedFist() const {
  return poseAtFraction(1.0, /*includeRootChain=*/false);
}

std::vector<std::map<std::string, std::vector<double>>> HandGestures::wave() const {
  const HandChainConfig *root = rootChain();
  const std::map<std::string, std::vector<double>> rest = restPose();

  if (root == nullptr || root->joints.empty()) return {rest};

  const HandJointConfig &wave_joint = root->joints[0];
  const double swing_one = to_radians(wave_joint.min_degrees + 0.8 * (wave_joint.max_degrees - wave_joint.min_degrees));
  const double swing_other =
      to_radians(wave_joint.min_degrees + 0.2 * (wave_joint.max_degrees - wave_joint.min_degrees));

  std::map<std::string, std::vector<double>> pose_a = rest;
  pose_a[root->name][0] = swing_one;
  std::map<std::string, std::vector<double>> pose_b = rest;
  pose_b[root->name][0] = swing_other;

  return {pose_a, pose_b, pose_a, rest};
}

} // namespace robotarm
