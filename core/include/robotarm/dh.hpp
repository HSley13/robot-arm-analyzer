#pragma once

#include <Eigen/Dense>

namespace robotarm {

// Convention: classic/standard Denavit-Hartenberg (not Craig's "modified" DH):
//   T_i^(i-1) = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
// Do not mix this with modified-DH tables - the matrix order differs and the
// mistake is a classic source of silent sign/axis bugs.
enum class JointType { Revolute,
                       Prismatic };

// One joint's DH parameters, plus which one is the controllable variable:
//   Revolute  -> theta is variable, a/alpha/d are fixed
//   Prismatic -> d is variable, a/alpha/theta are fixed
// The fixed theta/d fields act as a home-position offset added to the
// variable joint value (e.g. a joint that isn't zeroed at its mechanical
// home position).
struct DHParameter {
  double a = 0.0;     // link length
  double alpha = 0.0; // link twist (radians)
  double d = 0.0;     // link offset
  double theta = 0.0; // joint angle (radians)
  JointType type = JointType::Revolute;

  DHParameter() = default;
  DHParameter(double a_, double alpha_ = 0.0, double d_ = 0.0, double theta_ = 0.0, JointType type_ = JointType::Revolute)
      : a(a_), alpha(alpha_), d(d_), theta(theta_), type(type_) {}
};

// Builds the single-joint homogeneous transform T_i^(i-1) from the four DH
// parameters, using the classic DH convention above.
Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha);

} // namespace robotarm
