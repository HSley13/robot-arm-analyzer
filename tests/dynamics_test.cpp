// Verifies LinkInertial::rod() - the documented uniform-slender-rod
// simplification - against the textbook moment-of-inertia formula and
// the geometric placement it depends on (frame_i's local -X pointing
// back toward frame_{i-1} when the link has no twist) - and
// inverse_dynamics() (RNEA) against three independent, hand-derivable
// cases: the static (zero velocity/acceleration) case must match
// statics.hpp's Jacobian-transpose method exactly, since the roadmap
// states static analysis IS the dynamic equations at rest; a point mass
// spinning at constant velocity with no gravity needs zero actuator
// torque (elementary circular motion); and a point mass under pure
// angular acceleration with no gravity needs tau = m*L^2*alpha (the
// parallel-axis theorem, I = m*L^2 for a point mass at radius L).
#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/dynamics.hpp"
#include "robotarm/statics.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::inverse_dynamics;
using robotarm::LinkInertial;
using robotarm::LinkMass;
using robotarm::static_joint_torques;

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

TEST(Dynamics, RodInertiaMatchesTextbookFormula) {
  const double mass = 2.0;
  const double length = 1.0;
  const LinkInertial link = LinkInertial::rod(mass, length);

  const double expected_i_perp = mass * length * length / 12.0; // 0.1667
  EXPECT_DOUBLE_EQ(link.mass, mass);
  EXPECT_NEAR(link.inertia(0, 0), 0.0, 1e-12);          // along the rod: ~0
  EXPECT_NEAR(link.inertia(1, 1), expected_i_perp, 1e-12);
  EXPECT_NEAR(link.inertia(2, 2), expected_i_perp, 1e-12);
  EXPECT_NEAR(link.inertia(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(link.inertia(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(link.inertia(1, 2), 0.0, 1e-12);
}

TEST(Dynamics, RodCenterOfMassIsAtTheMidpoint) {
  const LinkInertial link = LinkInertial::rod(1.0, 0.5);
  // frame_i sits at the link's far end (see statics.hpp's LinkMass
  // convention) - the rod's center is half its length back along local
  // -X, i.e. (-length/2, 0, 0).
  EXPECT_NEAR(link.center_of_mass.x(), -0.25, 1e-12);
  EXPECT_NEAR(link.center_of_mass.y(), 0.0, 1e-12);
  EXPECT_NEAR(link.center_of_mass.z(), 0.0, 1e-12);
}

TEST(Dynamics, ScalesWithMassAndLengthSquared) {
  const LinkInertial base = LinkInertial::rod(1.0, 1.0);
  const LinkInertial double_mass = LinkInertial::rod(2.0, 1.0);
  const LinkInertial double_length = LinkInertial::rod(1.0, 2.0);

  EXPECT_NEAR(double_mass.inertia(1, 1), 2.0 * base.inertia(1, 1), 1e-12);
  EXPECT_NEAR(double_length.inertia(1, 1), 4.0 * base.inertia(1, 1), 1e-12);
}

TEST(Dynamics, StaticCaseMatchesJacobianTransposeExactly) {
  const double l1 = 1.0;
  const double l2 = 0.5;
  const ArmModel arm({DHParameter(l1), DHParameter(l2)});
  const std::vector<LinkMass> link_masses = {LinkMass(2.0), LinkMass(1.0)};
  // Zero offset, zero inertia tensor - matches LinkMass's point-mass-at-
  // frame-origin convention exactly, so the two methods describe the
  // identical physical system, not just a similar one.
  const std::vector<LinkInertial> link_inertials = {
      LinkInertial(2.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()),
      LinkInertial(1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()),
  };

  const std::vector<double> theta = {30.0 * kPi / 180.0, 45.0 * kPi / 180.0};
  const std::vector<double> zero_velocity = {0.0, 0.0};
  const std::vector<double> zero_acceleration = {0.0, 0.0};
  const double payload = 0.5;
  const Eigen::Vector3d gravity_down_y(0.0, -9.81, 0.0);

  const Eigen::VectorXd tau_static = static_joint_torques(arm, theta, link_masses, payload, gravity_down_y);
  const Eigen::VectorXd tau_rnea =
      inverse_dynamics(arm, theta, zero_velocity, zero_acceleration, link_inertials, payload, gravity_down_y);

  ASSERT_EQ(tau_static.size(), tau_rnea.size());
  EXPECT_NEAR(tau_rnea(0), tau_static(0), 1e-9);
  EXPECT_NEAR(tau_rnea(1), tau_static(1), 1e-9);
}

TEST(Dynamics, ConstantVelocityNoGravityNeedsZeroTorque) {
  // A point mass spinning at constant angular velocity about a fixed
  // pivot, with no gravity: the link supplies exactly the centripetal
  // force by tension alone, so the actuator needs zero torque - a
  // textbook circular-motion fact, independent of inverse_dynamics()'s
  // own implementation.
  const double length = 2.0;
  const double mass = 3.0;
  const ArmModel arm({DHParameter(length)});
  const std::vector<LinkInertial> links = {LinkInertial(mass, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero())};

  const std::vector<double> theta = {0.3};
  const std::vector<double> velocity = {1.7}; // rad/s, arbitrary nonzero
  const std::vector<double> acceleration = {0.0};
  const Eigen::Vector3d no_gravity = Eigen::Vector3d::Zero();

  const Eigen::VectorXd tau = inverse_dynamics(arm, theta, velocity, acceleration, links, 0.0, no_gravity);
  ASSERT_EQ(tau.size(), 1);
  EXPECT_NEAR(tau(0), 0.0, 1e-9);
}

TEST(Dynamics, PureAngularAccelerationMatchesParallelAxisTheorem) {
  // A point mass at radius L from the rotation axis, no gravity, zero
  // velocity: tau = I*alpha with I = m*L^2 (parallel axis theorem) -
  // elementary rotational F=ma, independent of this implementation.
  const double length = 1.5;
  const double mass = 4.0;
  const double alpha = 0.8; // rad/s^2
  const ArmModel arm({DHParameter(length)});
  const std::vector<LinkInertial> links = {LinkInertial(mass, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero())};

  const std::vector<double> theta = {0.0};
  const std::vector<double> velocity = {0.0};
  const std::vector<double> acceleration = {alpha};
  const Eigen::Vector3d no_gravity = Eigen::Vector3d::Zero();

  const Eigen::VectorXd tau = inverse_dynamics(arm, theta, velocity, acceleration, links, 0.0, no_gravity);
  ASSERT_EQ(tau.size(), 1);
  EXPECT_NEAR(tau(0), mass * length * length * alpha, 1e-9);
}

TEST(Dynamics, TorqueBreakdownSumsExactlyToTheDirectTotal) {
  // The whole point of decompose_torque() is that gravity+inertia+coriolis
  // must equal a DIRECTLY computed total, not just whatever the sum
  // happens to be - this is the actual verification, not the component
  // values individually.
  const ArmModel arm({DHParameter(1.0), DHParameter(0.5)});
  const std::vector<LinkInertial> links = {LinkInertial::rod(2.0, 1.0), LinkInertial::rod(1.0, 0.5)};

  const std::vector<double> theta = {0.6, -1.1};
  const std::vector<double> velocity = {1.3, -0.9};
  const std::vector<double> acceleration = {2.5, 1.7};
  const double payload = 1.5;
  const Eigen::Vector3d gravity(0.0, -9.81, 0.0);

  const robotarm::TorqueBreakdown breakdown =
      robotarm::decompose_torque(arm, theta, velocity, acceleration, links, payload, gravity);
  const Eigen::VectorXd direct_total = inverse_dynamics(arm, theta, velocity, acceleration, links, payload, gravity);

  ASSERT_EQ(breakdown.total.size(), direct_total.size());
  for (Eigen::Index i = 0; i < direct_total.size(); i++) {
    EXPECT_NEAR(breakdown.total(i), direct_total(i), 1e-9) << "joint " << i << " (direct total)";
    const double sum = breakdown.gravity(i) + breakdown.inertia(i) + breakdown.coriolis(i);
    EXPECT_NEAR(sum, direct_total(i), 1e-9) << "joint " << i << " (sum of components)";
  }
}

TEST(Dynamics, BreakdownComponentsAreZeroWhenTheirCauseIsAbsent) {
  // At rest (zero velocity, zero acceleration), inertia and Coriolis must
  // both vanish, leaving gravity as the entire total - directly testing
  // that each term is actually isolated, not just summing correctly.
  const ArmModel arm({DHParameter(1.0)});
  const std::vector<LinkInertial> links = {LinkInertial::rod(2.0, 1.0)};
  const std::vector<double> theta = {0.4};
  const std::vector<double> zero = {0.0};
  const Eigen::Vector3d gravity(0.0, -9.81, 0.0);

  const robotarm::TorqueBreakdown breakdown = robotarm::decompose_torque(arm, theta, zero, zero, links, 0.0, gravity);
  EXPECT_NEAR(breakdown.inertia(0), 0.0, 1e-12);
  EXPECT_NEAR(breakdown.coriolis(0), 0.0, 1e-12);
  EXPECT_NEAR(breakdown.gravity(0), breakdown.total(0), 1e-9);
}
