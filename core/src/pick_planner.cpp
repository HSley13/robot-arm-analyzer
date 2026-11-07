#include "robotarm/pick_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "robotarm/arm_model.hpp"
#include "robotarm/inverse_kinematics.hpp"
#include "robotarm/statics.hpp"
#include "robotarm/units.hpp"

namespace robotarm {

namespace {

double wrap_angle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::string joint_label(const ArmConfig &config, size_t index) {
  if (index < config.joints.size() && !config.joints[index].name.empty()) {
    return config.joints[index].name;
  }
  return "joint " + std::to_string(index + 1);
}

} // namespace

std::vector<std::vector<double>> planar_3r_ik(double a1, double a2, double a3, const Eigen::Vector2d &tip,
                                              double phi, const std::vector<double> &theta_offsets) {
  std::vector<std::vector<double>> solutions;

  // Step back from the tip along the hand to the wrist point.
  const Eigen::Vector2d wrist = tip - a3 * Eigen::Vector2d(std::cos(phi), std::sin(phi));

  // Two-link problem for the wrist point: law of cosines for the elbow.
  const double cos_elbow = (wrist.squaredNorm() - a1 * a1 - a2 * a2) / (2.0 * a1 * a2);
  constexpr double kTolerance = 1e-9;
  if (cos_elbow > 1.0 + kTolerance || cos_elbow < -1.0 - kTolerance) {
    return solutions; // wrist point out of the first two links' reach
  }
  const double elbow_magnitude = std::acos(std::clamp(cos_elbow, -1.0, 1.0));

  const double offset1 = theta_offsets.size() > 0 ? theta_offsets[0] : 0.0;
  const double offset2 = theta_offsets.size() > 1 ? theta_offsets[1] : 0.0;
  const double offset3 = theta_offsets.size() > 2 ? theta_offsets[2] : 0.0;

  for (int sign = 1; sign >= -1; sign -= 2) {
    const double elbow = sign * elbow_magnitude;
    // Shoulder: direction to the wrist, minus the angle the elbow bend
    // adds between the upper arm and that direction.
    const double shoulder = std::atan2(wrist.y(), wrist.x()) -
                            std::atan2(a2 * std::sin(elbow), a1 + a2 * std::cos(elbow));
    const double wrist_angle = phi - shoulder - elbow;

    solutions.push_back({wrap_angle(shoulder - offset1), wrap_angle(elbow - offset2), wrap_angle(wrist_angle - offset3)});
    if (elbow_magnitude < 1e-9) {
      break; // arm fully stretched: both "elbow ways" are the same pose
    }
  }
  return solutions;
}

std::optional<double> yaw_to_place_point_in_plane(const Eigen::Vector3d &point, double plane_offset, double &forward) {
  const double radius = std::hypot(point.x(), point.z());
  if (radius < std::abs(plane_offset) + 1e-9) {
    return std::nullopt;
  }
  forward = std::sqrt(radius * radius - plane_offset * plane_offset);

  // In the torso frame the point is at angle atan2(plane_offset, forward)
  // in the x-z plane; turning the torso by yaw moves that to
  // (that angle - yaw) in the world, which must equal the point's own
  // world angle. (Yaw is a right-handed turn about +Y, as in the 3D view.)
  const double local_angle = std::atan2(plane_offset, forward);
  const double world_angle = std::atan2(point.z(), point.x());
  return wrap_angle(local_angle - world_angle);
}

bool is_planar_3r(const ArmConfig &config) {
  if (config.joints.size() != 3) {
    return false;
  }
  for (const JointConfig &joint : config.joints) {
    if (joint.dh.type != JointType::Revolute || std::abs(joint.dh.alpha) > 1e-9 || std::abs(joint.dh.d) > 1e-9) {
      return false;
    }
  }
  return true;
}

Eigen::Vector2d shoulder_in_plane(const BodyGeometry &body, double hip_height, double hip_bend) {
  return {body.hip_to_shoulder * std::sin(hip_bend), hip_height + body.hip_to_shoulder * std::cos(hip_bend)};
}

namespace {

std::vector<JointLimit> limits_of(const ArmConfig &config) {
  std::vector<JointLimit> limits;
  for (const JointConfig &joint : config.joints) {
    limits.push_back({to_radians(joint.min_degrees), to_radians(joint.max_degrees)});
  }
  return limits;
}

bool within_limits(const ArmConfig &config, const std::vector<double> &q) {
  for (size_t i = 0; i < q.size(); i++) {
    const double degrees = to_degrees(q[i]);
    if (degrees < config.joints[i].min_degrees - 1e-6 || degrees > config.joints[i].max_degrees + 1e-6) {
      return false;
    }
  }
  return true;
}

// Every arm-joint solution that puts the tip at `tip_arm` with the hand at
// `phi_arm` (both in the arm's own frame) and respects the joints' travel.
// Three joints: the closed form, both elbow branches. Any other count:
// the numerical solver, started from each seed.
std::vector<std::vector<double>> arm_solutions(const ArmConfig &config, const ArmModel &arm,
                                               const Eigen::Vector2d &tip_arm, double phi_arm,
                                               const std::vector<std::vector<double>> &seeds) {
  std::vector<std::vector<double>> solutions;
  if (config.joints.size() == 3) {
    const std::vector<double> offsets = {config.joints[0].dh.theta, config.joints[1].dh.theta,
                                         config.joints[2].dh.theta};
    for (const std::vector<double> &q : planar_3r_ik(config.joints[0].dh.a, config.joints[1].dh.a,
                                                     config.joints[2].dh.a, tip_arm, phi_arm, offsets)) {
      if (within_limits(config, q)) {
        solutions.push_back(q);
      }
    }
    return solutions;
  }
  const std::vector<JointLimit> limits = limits_of(config);
  for (const std::vector<double> &seed : seeds) {
    const PlanarIkResult result = solve_planar_ik(arm, seed, tip_arm, &phi_arm, limits, 80);
    if (result.converged) {
      solutions.push_back(result.joint_values);
      break; // one good solution per pose is plenty for a sweep
    }
  }
  return solutions;
}

} // namespace

bool is_planar_revolute(const ArmConfig &config) {
  if (config.joints.empty()) {
    return false;
  }
  for (const JointConfig &joint : config.joints) {
    if (joint.dh.type != JointType::Revolute || std::abs(joint.dh.alpha) > 1e-9 || std::abs(joint.dh.d) > 1e-9) {
      return false;
    }
  }
  return true;
}

PickPlan plan_grasp(const ArmConfig &config, const BodyGeometry &body, const GraspTarget &target,
                    double current_hip_height, const Eigen::Vector3d &gravity) {
  PickPlan plan;
  if (!is_planar_revolute(config)) {
    plan.reason = "picking needs an arm whose joints all turn in one plane";
    return plan;
  }
  const size_t n = config.joints.size();

  // 1. Turn the waist so the object's center lies on the palm side of the
  //    arm's plane, palmar_offset away from it.
  const double plane_offset = body.shoulder_lateral - target.palmar_offset;
  double forward = 0.0;
  const std::optional<double> yaw = yaw_to_place_point_in_plane(target.grip_point, plane_offset, forward);
  if (!yaw.has_value() || forward < body.min_forward_reach) {
    plan.reason = "too close to the body to grasp";
    return plan;
  }
  if (*yaw < to_radians(body.min_yaw_degrees) || *yaw > to_radians(body.max_yaw_degrees)) {
    plan.reason = "behind the robot - outside the waist's turning range";
    return plan;
  }

  const ArmModel arm = to_arm_model(config);
  const std::vector<LinkMass> links = to_link_masses(config);

  // 2. Hand angles (in the world) to try: whatever the caller allows, or
  //    level only for upright objects, or a fan from well down to slightly up.
  std::vector<double> hand_angles = target.hand_angles;
  if (hand_angles.empty()) {
    hand_angles.push_back(0.0);
    if (!target.keep_hand_level) {
      for (int degrees = -80; degrees <= 30; degrees += 10) {
        if (degrees != 0) {
          hand_angles.push_back(to_radians(static_cast<double>(degrees)));
        }
      }
    }
  }

  // Seeds for the numerical solver (arms other than 3 joints): the last
  // solution found (neighbouring poses in the sweep are close), straight,
  // and folded like a "Z".
  std::vector<std::vector<double>> seeds(3, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; i++) {
    seeds[2][i] = (i == 0) ? -0.6 : (i % 2 == 1 ? 1.0 : -0.8);
  }

  // 3. Sweep lift height x hip bend x hand angle; keep the least-loaded pose.
  //    The closed form is cheap enough for a fine grid; the numerical solver
  //    gets a coarser one.
  const bool closed_form = n == 3;
  const double height_step = closed_form ? 0.02 : 0.05;
  const double bend_step_degrees = closed_form ? 5.0 : 7.5;

  double best_score = std::numeric_limits<double>::infinity();
  PickPlan best_overloaded;
  best_overloaded.peak_load_ratio = std::numeric_limits<double>::infinity();
  size_t overloaded_joint = 0;
  double overloaded_torque = 0.0;
  bool any_reachable = false;

  constexpr double kMinClearance = 0.03; // m, every joint and the tip kept this far above the floor
  for (double hip = body.min_hip_height; hip <= body.max_hip_height + 1e-9; hip += height_step) {
    for (double bend_degrees = 0.0; bend_degrees <= body.max_hip_bend_degrees + 1e-9;
         bend_degrees += bend_step_degrees) {
      const double bend = to_radians(bend_degrees);
      const Eigen::Vector2d shoulder = shoulder_in_plane(body, hip, bend);
      // Leaning forward by `bend` turns the arm's base frame by -bend about
      // the lateral axis, so world -> arm frame is a turn by +bend.
      const Eigen::Rotation2Dd world_to_arm(bend);

      for (const double phi_world : hand_angles) {
        // The arm's end-effector point sits forward_offset behind the
        // object's center, along the hand.
        const Eigen::Vector2d tip_world(forward - target.forward_offset * std::cos(phi_world),
                                        target.grip_point.y() - target.forward_offset * std::sin(phi_world));
        const Eigen::Vector2d tip_arm = world_to_arm * (tip_world - shoulder);
        const double phi_arm = phi_world + bend;

        for (const std::vector<double> &q : arm_solutions(config, arm, tip_arm, phi_arm, seeds)) {
          seeds[0] = q;

          // The same arm seen with gravity's frame: shoulder angle minus the
          // bend (see the header comment). Used for floor clearance and torques.
          std::vector<double> q_world = q;
          q_world[0] -= bend;

          bool clears_floor = true;
          for (const Eigen::Matrix4d &frame : arm.forward_kinematics(q_world)) {
            if (shoulder.y() + frame(1, 3) < kMinClearance) {
              clears_floor = false;
            }
          }
          if (!clears_floor) {
            continue;
          }
          any_reachable = true;

          const Eigen::VectorXd torques = static_joint_torques(arm, q_world, links, target.mass, gravity);
          double ratio = 0.0;
          size_t worst = 0;
          for (size_t i = 0; i < n; i++) {
            const double limit = config.joints[i].torque_limit;
            const double r = limit > 0.0 ? std::abs(torques(static_cast<Eigen::Index>(i))) / limit : 0.0;
            if (r > ratio) {
              ratio = r;
              worst = i;
            }
          }

          PickPlan candidate;
          candidate.pose.yaw = *yaw;
          candidate.pose.hip_height = hip;
          candidate.pose.hip_bend = bend;
          candidate.pose.arm = q;
          candidate.hand_angle = phi_world;
          candidate.peak_load_ratio = ratio;

          if (ratio > 1.0) {
            if (ratio < best_overloaded.peak_load_ratio) {
              best_overloaded = candidate;
              overloaded_joint = worst;
              overloaded_torque = std::abs(torques(static_cast<Eigen::Index>(worst)));
            }
            continue;
          }

          // Lightest load first; then prefer standing upright, the first
          // (preferred) hand angle, and not moving the lift far.
          const double score = ratio + 0.25 * bend + 0.3 * std::abs(phi_world - hand_angles.front()) +
                               0.15 * std::abs(hip - current_hip_height);
          if (score < best_score) {
            best_score = score;
            plan = candidate;
            plan.ok = true;
          }
        }
      }
    }
  }

  if (plan.ok) {
    return plan;
  }

  if (!any_reachable) {
    plan.reason = "out of reach";
    return plan;
  }

  // Reachable, but every way of reaching it overloads something: report
  // the least-bad pose's worst joint.
  char message[160];
  std::snprintf(message, sizeof(message), "too heavy - %s would need %.0f N*m (limit %.0f)",
                joint_label(config, overloaded_joint).c_str(), overloaded_torque,
                config.joints[overloaded_joint].torque_limit);
  best_overloaded.ok = false;
  best_overloaded.reason = message;
  return best_overloaded;
}

} // namespace robotarm
