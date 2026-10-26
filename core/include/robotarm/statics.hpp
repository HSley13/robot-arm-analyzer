#pragma once

#include <vector>

#include <Eigen/Dense>

#include "robotarm/arm_model.hpp"

namespace robotarm {

// Computes the 6xN geometric Jacobian mapping joint velocities to the
// linear (rows 0-2) and angular (rows 3-5) velocity, expressed in the base
// frame, of the ORIGIN of joint `target_joint`'s frame (1-indexed; pass
// arm.num_joints() for the end-effector). Columns for joints after
// target_joint are zero - a joint can't move a point that sits on an
// earlier link in the chain.
Eigen::MatrixXd geometric_jacobian(const ArmModel &arm, const std::vector<double> &joint_values, size_t target_joint);

// One link's mass, treated as a point mass located at its own joint frame's
// origin (the far end of the link, where the next joint attaches) - not an
// arbitrary center-of-mass offset. A documented simplification matching the
// project's original 2D "sum of moments" formula, extended into 3D; a
// per-link center-of-mass offset can be layered in later without changing
// the method.
struct LinkMass {
  double mass = 0.0; // kg

  LinkMass() = default;
  explicit LinkMass(double mass_) : mass(mass_) {}
};

// Static joint torques via the Jacobian-transpose method - the textbook 3D
// generalization of "sum of moments about each joint", and, per the
// roadmap, exactly the dynamic equations (Phase 2) with zero velocity and
// zero acceleration. Sums every link's gravity load (applied at its own
// joint-frame origin) plus the payload's weight at the end-effector.
//
// Sign convention (stated explicitly, per the roadmap's DH-convention
// warning about silent sign bugs): returns the ACTUATOR torque needed to
// hold static equilibrium against the given load, not the load's own
// moment on the joints - e.g. a horizontal arm with a weight at the tip
// returns a POSITIVE torque (the effort needed to hold it up), not the
// negative moment gravity itself exerts.
//
// `gravity` is the gravitational acceleration vector expressed in the base
// frame (default: -9.81 m/s^2 along base Z, i.e. base Z is "up").
Eigen::VectorXd static_joint_torques(const ArmModel &arm, const std::vector<double> &joint_values,
                                      const std::vector<LinkMass> &links, double payload_mass,
                                      const Eigen::Vector3d &gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

} // namespace robotarm
