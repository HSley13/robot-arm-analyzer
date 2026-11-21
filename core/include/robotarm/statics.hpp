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

// One link's mass and where it sits: `center_of_mass` is expressed in the
// link's own joint frame (frame_i, whose origin is the far end of the
// link, where the next joint attaches) - the same frame and the same
// point LinkInertial::center_of_mass uses, so statics and dynamics agree
// on where every link's weight acts. The default (zero) is a point mass
// at the joint-frame origin, the project's original 2D "sum of moments"
// simplification.
struct LinkMass {
  double mass = 0.0;                                        // kg
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero(); // meters, in frame_i

  LinkMass() = default;
  explicit LinkMass(double mass_, const Eigen::Vector3d &center_of_mass_ = Eigen::Vector3d::Zero())
      : mass(mass_), center_of_mass(center_of_mass_) {}
};

// Static joint torques via the Jacobian-transpose method - the textbook 3D
// generalization of "sum of moments about each joint", and, per the
// roadmap, exactly the dynamic equations (Phase 2) with zero velocity and
// zero acceleration. Sums every link's gravity load (applied at its own
// center of mass, so a LinkMass built from the same config as a
// LinkInertial gives exactly inverse_dynamics() at rest) plus the
// payload's weight at the end-effector.
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
Eigen::VectorXd static_joint_torques(const ArmModel &arm, const std::vector<double> &joint_values, const std::vector<LinkMass> &links, double payload_mass, const Eigen::Vector3d &gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

} // namespace robotarm
