// Offline tool (not shipped in the app): sweeps random joint values and
// payload masses through the *exact* physics engine and writes
// (input -> output) pairs to CSV, which train_surrogate.py then trains an
// MLP on. Per the plan: "the exact formula above *is* the data generator" -
// no separate data-collection step, no duplicate physics implementation in
// Python.
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "robotarm/arm_config.hpp"
#include "robotarm/statics.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
// No per-joint range in the config schema yet (Phase 1 simplification) -
// revolute joints sweep the full normalized range, prismatic joints sweep
// a generic default travel.
constexpr double kPrismaticMaxMeters = 0.3;
constexpr double kMaxPayloadKg = 5.0;

} // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: generate_training_data <arm_config.json> <output.csv> <num_samples> [seed] [gx gy gz]\n"
               << "  gravity defaults to (0, 0, -9.81) - base Z is 'up'. For an arm whose\n"
               << "  joints all rotate about Z (no link twist), that default is perpendicular\n"
               << "  to the arm's whole plane of motion and produces an all-zero training\n"
               << "  signal (rotating about the gravity axis never changes height) - pass an\n"
               << "  in-plane gravity vector explicitly for that case, e.g. 0 -9.81 0.\n";
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
  const std::vector<robotarm::LinkMass> links = robotarm::to_link_masses(config);
  const size_t n = arm.num_joints();

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> revolute_dist(-kPi, kPi);
  std::uniform_real_distribution<double> prismatic_dist(0.0, kPrismaticMaxMeters);
  std::uniform_real_distribution<double> payload_dist(0.0, kMaxPayloadKg);

  std::ofstream out(output_path);
  if (!out) {
    std::cerr << "could not open output file '" << output_path << "'\n";
    return 1;
  }

  for (size_t i = 0; i < n; i++) out << "joint" << i << ",";
  out << "payload,";
  for (size_t i = 0; i < n; i++) out << "tau" << i << (i + 1 < n ? "," : "\n");

  std::vector<double> joint_values(n);
  for (int sample = 0; sample < num_samples; sample++) {
    for (size_t i = 0; i < n; i++) {
      joint_values[i] =
          (arm.joints()[i].type == robotarm::JointType::Revolute) ? revolute_dist(rng) : prismatic_dist(rng);
    }
    const double payload = payload_dist(rng);
    const Eigen::VectorXd tau = robotarm::static_joint_torques(arm, joint_values, links, payload, gravity);

    for (size_t i = 0; i < n; i++) out << joint_values[i] << ",";
    out << payload << ",";
    for (size_t i = 0; i < n; i++) out << tau(static_cast<Eigen::Index>(i)) << (i + 1 < n ? "," : "\n");
  }

  std::cout << "wrote " << num_samples << " samples (" << n << " joints) to " << output_path << "\n";
  return 0;
}
