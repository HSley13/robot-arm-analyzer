#include "robotarm/dynamics.hpp"

#include <cassert>

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

Eigen::VectorXd inverse_dynamics(const ArmModel &arm, const std::vector<double> &joint_values,
                                  const std::vector<double> &joint_velocities,
                                  const std::vector<double> &joint_accelerations,
                                  const std::vector<LinkInertial> &links, double payload_mass,
                                  const Eigen::Vector3d &gravity) {
  const size_t n = arm.num_joints();
  assert(joint_velocities.size() == n);
  assert(joint_accelerations.size() == n);
  assert(links.size() == n);

  const std::vector<Eigen::Matrix4d> frames = arm.forward_kinematics(joint_values); // T_1^0 .. T_n^0

  std::vector<Eigen::Vector3d> origin(n + 1);
  std::vector<Eigen::Matrix3d> rotation(n + 1);
  origin[0] = Eigen::Vector3d::Zero();
  rotation[0] = Eigen::Matrix3d::Identity();
  for (size_t i = 1; i <= n; i++) {
    rotation[i] = frames[i - 1].block<3, 3>(0, 0);
    origin[i] = frames[i - 1].block<3, 1>(0, 3);
  }

  // Forward pass: angular velocity/acceleration and each frame origin's
  // linear acceleration, for frames 0..n, all expressed in the base
  // frame. Gravity is folded in via the standard "fictitious base
  // acceleration" trick: accel[0] = -gravity makes the recursion behave
  // as if the base were accelerating upward against gravity - equivalent
  // (same principle as "everything in a sealed, accelerating elevator
  // feels gravity") to every link feeling its own weight, with no
  // separate gravity term needed anywhere else.
  std::vector<Eigen::Vector3d> omega(n + 1), omega_dot(n + 1), accel(n + 1);
  omega[0] = Eigen::Vector3d::Zero();
  omega_dot[0] = Eigen::Vector3d::Zero();
  accel[0] = -gravity;

  for (size_t i = 1; i <= n; i++) {
    const Eigen::Vector3d z = rotation[i - 1].col(2); // frame_{i-1}'s z-axis = joint i's own axis
    const Eigen::Vector3d r = origin[i] - origin[i - 1];
    const double qdot = joint_velocities[i - 1];
    const double qddot = joint_accelerations[i - 1];

    // frame_i's origin is a material point rigidly attached to "link i"
    // (the body joint i drives) - its acceleration transfer from
    // frame_{i-1} must use link i's OWN omega[i]/omega_dot[i] (which
    // already includes joint i's contribution), not the previous link's.
    // Using omega[i-1]/omega_dot[i-1] here made a rotating point mass's
    // own acceleration come out zero - caught by
    // PureAngularAccelerationMatchesParallelAxisTheorem. The Coriolis-like
    // term INSIDE the omega_dot update below is different: it accounts
    // for the joint's axis itself being carried along by the previous
    // link's motion, so it correctly keeps using omega[i-1].
    if (arm.joints()[i - 1].type == JointType::Revolute) {
      omega[i] = omega[i - 1] + z * qdot;
      omega_dot[i] = omega_dot[i - 1] + z * qddot + omega[i - 1].cross(z * qdot);
      accel[i] = accel[i - 1] + omega_dot[i].cross(r) + omega[i].cross(omega[i].cross(r));
    } else {
      omega[i] = omega[i - 1];
      omega_dot[i] = omega_dot[i - 1];
      accel[i] = accel[i - 1] + omega_dot[i].cross(r) + omega[i].cross(omega[i].cross(r)) +
                 2.0 * omega[i].cross(z * qdot) + z * qddot;
    }
  }

  // Backward pass. Moments are accumulated about ONE FIXED point - the
  // base origin - rather than re-based link by link: summing several
  // forces/moments that are all already expressed about the same point
  // needs no transfer formula, which is what makes this simple and hard
  // to get backwards. Only the very last step (per joint, since
  // frame_{i-1}'s origin differs per joint) transfers the accumulated
  // moment to a point that actually lies ON that joint's own axis line,
  // before projecting onto the axis direction.
  Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d total_moment = Eigen::Vector3d::Zero(); // about the base origin

  if (payload_mass != 0.0) {
    // A zero-offset, zero-inertia point mass rigidly at frame_n - the
    // same point-mass convention statics.hpp's LinkMass uses.
    const Eigen::Vector3d payload_force = payload_mass * accel[n];
    total_force += payload_force;
    total_moment += origin[n].cross(payload_force);
  }

  Eigen::VectorXd tau(static_cast<Eigen::Index>(n));

  for (size_t i = n; i >= 1; i--) {
    const Eigen::Vector3d com_offset = rotation[i] * links[i - 1].center_of_mass;
    const Eigen::Vector3d com_world = origin[i] + com_offset;
    const Eigen::Matrix3d inertia_world = rotation[i] * links[i - 1].inertia * rotation[i].transpose();

    const Eigen::Vector3d a_com =
        accel[i] + omega_dot[i].cross(com_offset) + omega[i].cross(omega[i].cross(com_offset));
    const Eigen::Vector3d force_i = links[i - 1].mass * a_com;
    const Eigen::Vector3d euler_i = inertia_world * omega_dot[i] + omega[i].cross(inertia_world * omega[i]);

    total_force += force_i;
    total_moment += com_world.cross(force_i) + euler_i;

    // Transfer from "about the base origin" to "about a point on joint
    // i's own axis" (frame_{i-1}'s origin), then project onto the axis
    // direction z_{i-1}.
    const Eigen::Vector3d z = rotation[i - 1].col(2);
    const Eigen::Vector3d moment_on_axis = total_moment - origin[i - 1].cross(total_force);

    tau(static_cast<Eigen::Index>(i - 1)) =
        (arm.joints()[i - 1].type == JointType::Revolute) ? moment_on_axis.dot(z) : total_force.dot(z);
  }

  return tau;
}

} // namespace robotarm
