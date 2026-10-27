#pragma once

#include <vector>

#include <Eigen/Dense>

#include "robotarm/arm_model.hpp"

namespace robotarm {

// One link's full inertial properties for dynamics (Phase 2) - a strict
// generalization of statics.hpp's LinkMass point-mass simplification.
// `center_of_mass` and `inertia` are both expressed in the link's own
// joint frame (frame_i, the SAME frame LinkMass's point mass sits at in
// Phase 1 statics) - not the base frame, and not necessarily at that
// frame's origin.
struct LinkInertial {
  double mass = 0.0;
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero(); // meters, in frame_i
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();        // kg*m^2, about center_of_mass, in frame_i

  LinkInertial() = default;
  LinkInertial(double mass_, const Eigen::Vector3d &center_of_mass_, const Eigen::Matrix3d &inertia_)
      : mass(mass_), center_of_mass(center_of_mass_), inertia(inertia_) {}

  // A documented simplifying assumption (the same kind the roadmap found
  // NovaSolver states plainly rather than hiding): a uniform slender rod
  // of the given length/mass, lying along frame_i's local -X axis. This
  // only places the rod correctly when the link has no twist
  // (alpha_i = 0), since that's what makes frame_i's local -X point back
  // toward frame_{i-1} - for a twisted link, construct LinkInertial
  // directly with the real geometry instead of using this factory.
  static LinkInertial rod(double mass, double length);
};

// Full inverse dynamics via the Recursive Newton-Euler Algorithm (RNEA):
// the joint torque/force needed to produce the given motion
// (joint_velocities, joint_accelerations - same units as joint_values:
// rad/s and rad/s^2 for Revolute, m/s and m/s^2 for Prismatic) under
// gravity and an optional point-mass payload at the end-effector.
//
// Setting joint_velocities and joint_accelerations to all zero collapses
// this exactly to static_joint_torques() (verified in dynamics_test.cpp)
// - per the roadmap, static analysis IS the dynamic equations with zero
// velocity and zero acceleration, so Phase 1's work isn't thrown away.
//
// Implementation note: every quantity (angular velocity/acceleration,
// linear acceleration, force, moment) is expressed in the BASE frame
// throughout, and moments are accumulated about one FIXED point (the
// base origin) rather than being re-based link by link - a per-link
// reference point still requires one careful final transfer to the
// specific joint axis being queried, which is where an earlier draft of
// this function had a sign/reference-point bug.
Eigen::VectorXd inverse_dynamics(const ArmModel &arm, const std::vector<double> &joint_values,
                                  const std::vector<double> &joint_velocities,
                                  const std::vector<double> &joint_accelerations,
                                  const std::vector<LinkInertial> &links, double payload_mass,
                                  const Eigen::Vector3d &gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

} // namespace robotarm
