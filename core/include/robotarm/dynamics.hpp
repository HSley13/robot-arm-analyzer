#pragma once

#include <Eigen/Dense>

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

} // namespace robotarm
