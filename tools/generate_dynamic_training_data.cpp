// Offline tool (not shipped in the app): the dynamic-case counterpart to
// generate_training_data.cpp. Sweeps random joint values, velocities,
// accelerations, and payload masses through the *exact* RNEA engine
// (inverse_dynamics()) and writes (input -> output) pairs to CSV, which
// train_dynamic_surrogate.py trains an MLP on. Same principle as the
// static generator: the exact physics engine IS the data generator, so
// there's no duplicate implementation to keep in sync.
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "robotarm/arm_config.hpp"
#include "robotarm/dynamics.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
// Training-domain ranges - a documented choice, not a universal truth
// (same spirit as the static generator's kPrismaticMaxMeters/kMaxPayloadKg):
// the surrogate will only be accurate for inputs inside whatever range it
// was trained on.
constexpr double kPrismaticMaxMeters = 0.3;
constexpr double kMaxPayloadKg = 5.0;
constexpr double kRevoluteMaxVelocity = 2.0;      // rad/s
constexpr double kPrismaticMaxVelocity = 0.5;     // m/s
constexpr double kRevoluteMaxAcceleration = 5.0;  // rad/s^2
constexpr double kPrismaticMaxAcceleration = 1.0; // m/s^2

} // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: generate_dynamic_training_data <arm_config.json> <output.csv> <num_samples> "
                 "[seed] [gx gy gz]\n"
              << "  gravity defaults to (0, 0, -9.81) - base Z is 'up'. See\n"
              << "  generate_training_data's own usage text for why an arm with no link\n"
              << "  twist needs an in-plane override instead, e.g. 0 -9.81 0.\n";
    return 1;
  }

  const std::string config_path = argv[1];
  const std::string output_path = argv[2];
  const int num_samples = std::stoi(argv[3]);
  const unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : std::random_device{}();
  const Eigen::Vector3d gravity = argc > 7 ? Eigen::Vector3d(std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]))
                                            : Eigen::Vector3d(0.0, 0.0, -9.81);

  const robotarm::ArmConfig config = robotarm::load_arm_config(config_path);
  const robotarm::ArmModel arm = robotarm::to_arm_model(config);
  const std::vector<robotarm::LinkInertial> links = robotarm::to_link_inertials(config);
  const size_t n = arm.num_joints();

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> revolute_pos_dist(-kPi, kPi);
  std::uniform_real_distribution<double> prismatic_pos_dist(0.0, kPrismaticMaxMeters);
  std::uniform_real_distribution<double> revolute_vel_dist(-kRevoluteMaxVelocity, kRevoluteMaxVelocity);
  std::uniform_real_distribution<double> prismatic_vel_dist(-kPrismaticMaxVelocity, kPrismaticMaxVelocity);
  std::uniform_real_distribution<double> revolute_accel_dist(-kRevoluteMaxAcceleration, kRevoluteMaxAcceleration);
  std::uniform_real_distribution<double> prismatic_accel_dist(-kPrismaticMaxAcceleration, kPrismaticMaxAcceleration);
  std::uniform_real_distribution<double> payload_dist(0.0, kMaxPayloadKg);

  std::ofstream out(output_path);
  if (!out) {
    std::cerr << "could not open output file '" << output_path << "'\n";
    return 1;
  }

  for (size_t i = 0; i < n; i++) out << "joint" << i << ",";
  for (size_t i = 0; i < n; i++) out << "vel" << i << ",";
  for (size_t i = 0; i < n; i++) out << "accel" << i << ",";
  out << "payload,";
  for (size_t i = 0; i < n; i++) out << "tau" << i << (i + 1 < n ? "," : "\n");

  std::vector<double> joint_values(n), joint_velocities(n), joint_accelerations(n);
  for (int sample = 0; sample < num_samples; sample++) {
    for (size_t i = 0; i < n; i++) {
      const bool revolute = arm.joints()[i].type == robotarm::JointType::Revolute;
      joint_values[i] = revolute ? revolute_pos_dist(rng) : prismatic_pos_dist(rng);
      joint_velocities[i] = revolute ? revolute_vel_dist(rng) : prismatic_vel_dist(rng);
      joint_accelerations[i] = revolute ? revolute_accel_dist(rng) : prismatic_accel_dist(rng);
    }
    const double payload = payload_dist(rng);
    const Eigen::VectorXd tau =
        robotarm::inverse_dynamics(arm, joint_values, joint_velocities, joint_accelerations, links, payload, gravity);

    for (size_t i = 0; i < n; i++) out << joint_values[i] << ",";
    for (size_t i = 0; i < n; i++) out << joint_velocities[i] << ",";
    for (size_t i = 0; i < n; i++) out << joint_accelerations[i] << ",";
    out << payload << ",";
    for (size_t i = 0; i < n; i++) out << tau(static_cast<Eigen::Index>(i)) << (i + 1 < n ? "," : "\n");
  }

  std::cout << "wrote " << num_samples << " samples (" << n << " joints) to " << output_path << "\n";
  return 0;
}
