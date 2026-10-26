// Verifies the Jacobian-transpose static torque method: the geometric
// Jacobian itself in isolation, a hand-computable single-joint case, and a
// virtual-work cross-check - the computed torque must equal the numerical
// gradient of total gravitational potential energy with respect to each
// joint angle, the standard implementation-independent way to validate a
// static-torque Jacobian-transpose method.
#include <vector>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/statics.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::geometric_jacobian;
using robotarm::LinkMass;
using robotarm::static_joint_torques;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;
const Eigen::Vector3d kGravityDownY(0.0, -kG, 0.0);
} // namespace

TEST(Statics, GeometricJacobianSingleJoint) {
  // a=L, alpha=0, theta=0: joint axis z0=(0,0,1) through the origin, target
  // at (L,0,0). Jv = z x (target - origin) = (0,L,0), Jw = z = (0,0,1).
  const double length = 2.0;
  ArmModel arm({DHParameter(length)});
  const Eigen::MatrixXd jacobian = geometric_jacobian(arm, {0.0}, 1);

  ASSERT_EQ(jacobian.rows(), 6);
  ASSERT_EQ(jacobian.cols(), 1);
  EXPECT_NEAR(jacobian(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(jacobian(1, 0), length, 1e-9);
  EXPECT_NEAR(jacobian(2, 0), 0.0, 1e-9);
  EXPECT_NEAR(jacobian(3, 0), 0.0, 1e-9);
  EXPECT_NEAR(jacobian(4, 0), 0.0, 1e-9);
  EXPECT_NEAR(jacobian(5, 0), 1.0, 1e-9);
}

TEST(Statics, SingleJointHorizontalLoadMatchesHandCalc) {
  // Holding a horizontal arm of length L with a point mass m at its tip
  // takes torque m*g*L - the textbook statics check from the original
  // plan, now computed via the general 3D Jacobian-transpose method.
  const double length = 2.0;
  const double mass = 3.0;
  ArmModel arm({DHParameter(length)});

  const Eigen::VectorXd tau =
      static_joint_torques(arm, {0.0}, {LinkMass(mass)}, /*payload_mass=*/0.0, kGravityDownY);

  ASSERT_EQ(tau.size(), 1);
  EXPECT_NEAR(tau(0), mass * kG * length, 1e-9);
}

TEST(Statics, MatchesPotentialEnergyGradient) {
  const double l1 = 1.0;
  const double l2 = 0.5;
  const double m1 = 2.0;
  const double m2 = 1.0;
  const double payload = 0.5;
  ArmModel arm({DHParameter(l1), DHParameter(l2)});
  const std::vector<LinkMass> links = {LinkMass(m1), LinkMass(m2)};

  const std::vector<double> theta = {30.0 * kPi / 180.0, 45.0 * kPi / 180.0};

  // Total gravitational PE: link 1's point mass plus link 2's and the
  // payload's (both colocated at the end-effector), each times height
  // (the Y coordinate here, since gravity is along -Y).
  auto potential_energy = [&](const std::vector<double> &q) {
    const std::vector<Eigen::Matrix4d> poses = arm.forward_kinematics(q);
    const double height1 = poses[0](1, 3);
    const double height2 = poses[1](1, 3);
    return m1 * kG * height1 + (m2 + payload) * kG * height2;
  };

  const double h = 1e-6;
  std::vector<double> expected_tau(2);
  for (size_t i = 0; i < 2; i++) {
    std::vector<double> q_plus = theta;
    std::vector<double> q_minus = theta;
    q_plus[i] += h;
    q_minus[i] -= h;
    expected_tau[i] = (potential_energy(q_plus) - potential_energy(q_minus)) / (2.0 * h);
  }

  const Eigen::VectorXd tau = static_joint_torques(arm, theta, links, payload, kGravityDownY);

  ASSERT_EQ(tau.size(), 2);
  EXPECT_NEAR(tau(0), expected_tau[0], 1e-5);
  EXPECT_NEAR(tau(1), expected_tau[1], 1e-5);
}
