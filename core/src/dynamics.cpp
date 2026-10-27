#include "robotarm/dynamics.hpp"

namespace robotarm {

LinkInertial LinkInertial::rod(double mass, double length) {
  // Uniform thin rod about its center: I_perp = m*L^2/12 for the two axes
  // perpendicular to the rod, ~0 for the axis along the rod (no radius).
  const double i_perp = mass * length * length / 12.0;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(1, 1) = i_perp;
  inertia(2, 2) = i_perp;
  return LinkInertial(mass, Eigen::Vector3d(-length / 2.0, 0.0, 0.0), inertia);
}

} // namespace robotarm
