#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "robotarm/arm_config.hpp"

namespace robotarm {

// Planning for "walk the hand to an object and grasp it" with a humanoid
// whose arm moves in one vertical plane (the arm the rest of this project
// analyzes), mounted on a body that can do two more things:
//
//   * turn at the waist (yaw about the vertical axis), and
//   * raise/lower the whole torso on its lift column (shoulder height).
//
// Why those two and not, say, a shoulder that swings sideways: neither
// one changes the gravity torque at any arm joint. Turning about the
// gravity axis keeps the arm's plane vertical, and lifting is a pure
// translation - so every torque/safety number the analyzer shows stays
// exact while the body moves. (Dynamic mode still treats the waist as
// stationary; a fast waist turn would add sideways inertial loads that
// the planar model doesn't see.)
//
// World frame used here (the 3D view's): +Y up, the floor at y = 0, the
// waist axis through the origin. At yaw 0 the torso faces +X and the
// right shoulder sits at +Z (the robot's right). The arm's own plane is
// the torso's x-y plane shifted to z = shoulder_lateral.
//
// The hand grasps from the side in the shipped config: palm facing the
// body's midline (torso -Z), fingers along the hand link, thumb up. So a
// grasped object's center sits `palmar_offset` meters toward -Z of the
// arm's plane and `forward_offset` meters along the hand past the arm's
// end-effector point.

struct BodyGeometry {
  double shoulder_lateral = 0.20; // m, shoulder joint's distance from the waist axis
  double hip_to_shoulder = 0.45;  // m, hip joint to shoulder joint along the torso
  double min_hip_height = 0.25;   // m above the floor, lift fully down
  double max_hip_height = 0.95;   // m, lift fully up
  double max_hip_bend_degrees = 75.0;
  double min_yaw_degrees = -160.0;
  double max_yaw_degrees = 160.0;
  double min_forward_reach = 0.16; // m, the hand can't work closer to the body's axis than this
};

// Where the body + arm are: waist yaw (rad, about +Y), hip height (m),
// hip bend (rad, positive = leaning forward), and the arm's own joint
// values (radians, relative to the torso - same convention as ArmModel).
struct BodyPose {
  double yaw = 0.0;
  double hip_height = 0.8;
  double hip_bend = 0.0;
  std::vector<double> arm;
};

// Shoulder position in the arm's plane (x forward from the waist axis,
// y up from the floor) for a given hip height and bend.
Eigen::Vector2d shoulder_in_plane(const BodyGeometry &body, double hip_height, double hip_bend);

struct GraspTarget {
  Eigen::Vector3d grip_point = Eigen::Vector3d::Zero(); // world, where the object's grip center must end up
  double palmar_offset = 0.04;  // m, object center's distance out of the arm plane, toward the palm
  double forward_offset = 0.0;  // m, object center past the arm's end-effector point, along the hand
  double mass = 0.0;            // kg, checked against the joints' torque limits while held
  // Upright objects (bottles, cans, a standing dumbbell) must be held with
  // the hand level - thumb straight up - or they'd tip in the hand. A ball
  // doesn't care, so the planner may tilt the hand to reach it.
  bool keep_hand_level = true;
  // Explicit world hand angles to try (radians, 0 = level, -pi/2 = pointing
  // straight down), most preferred first. Overrides keep_hand_level when
  // non-empty - e.g. a parallel gripper can take an upright object either
  // from above or from the side.
  std::vector<double> hand_angles;
};

struct PickPlan {
  bool ok = false;
  std::string reason; // human-readable, set whenever ok is false
  BodyPose pose;
  double hand_angle = 0.0;      // rad, the hand link's direction in the world (0 = level)
  double peak_load_ratio = 0.0; // max |torque| / limit over the arm joints, holding `mass`
};

// Closed-form inverse kinematics for a planar three-revolute arm with the
// hand's direction specified as well as its position.
//
// Why closed form: three planar joints have exactly three degrees of
// freedom and "tip at (x, y) pointing at angle phi" is exactly three
// constraints, so there's no redundancy to resolve iteratively. Walk back
// from the tip along the hand by a3 to get the wrist point W; the first
// two links must reach W, which is the classic two-link problem (law of
// cosines gives the elbow, two atan2s the shoulder); the wrist then takes
// whatever angle is left over so the three angles sum to phi.
//
// a1..a3 are the link lengths; theta_offsets are the joints' fixed DH
// theta offsets (subtracted out so the returned values are joint values,
// not absolute angles). Returns zero, one or two solutions (elbow one way
// or the other), unfiltered by joint limits.
std::vector<std::vector<double>> planar_3r_ik(double a1, double a2, double a3, const Eigen::Vector2d &tip,
                                              double phi, const std::vector<double> &theta_offsets = {0, 0, 0});

// The waist yaw that puts a world point `point` (only its x/z matter) at
// lateral offset `plane_offset` from the waist axis, measured in the
// torso's own frame (torso z). Also returns, via `forward`, that point's
// distance in front of the waist along the torso's x. Empty if the point
// is closer to the waist axis than |plane_offset| (inside the body).
std::optional<double> yaw_to_place_point_in_plane(const Eigen::Vector3d &point, double plane_offset, double &forward);

// True if `config` is exactly three revolute joints, all in one plane
// (alpha = 0, d = 0) - the case with a closed-form solution.
bool is_planar_3r(const ArmConfig &config);

// True if every joint is revolute and in one plane, any number of joints -
// the arms plan_grasp() handles (three joints in closed form, others with
// solve_planar_ik()).
bool is_planar_revolute(const ArmConfig &config);

// Finds a body pose that puts the hand around `target`.
//
// Searches hip heights across the lift's range, hip bends from upright to
// max_hip_bend_degrees (and, for objects that don't need a level hand, a
// fan of hand angles), keeps the solutions that respect every joint's
// travel and keep the arm off the floor, and picks the one that loads the
// joints least while holding target.mass - with small preferences for
// standing upright and not moving the lift far from `current_hip_height`. If nothing is reachable, or
// everything reachable overloads a joint, ok is false and `reason` says
// which ("out of reach", "too heavy - shoulder would need 112 N*m of 90").
PickPlan plan_grasp(const ArmConfig &config, const BodyGeometry &body, const GraspTarget &target,
                    double current_hip_height,
                    const Eigen::Vector3d &gravity = Eigen::Vector3d(0.0, -9.81, 0.0));

} // namespace robotarm
