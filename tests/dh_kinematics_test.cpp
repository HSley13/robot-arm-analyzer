// Verifies DH-based forward kinematics: identity/rotation/twist sanity
// checks, plus the textbook verification the roadmap calls for - a planar
// 2-link arm's end-effector position checked against the classic trig
// formula (x = sum L_k cos(theta_1+...+theta_k), y analogous).
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::JointType;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

bool close(double a, double b, double tol = kTol) { return std::abs(a - b) < tol; }

void test_identity_transform() {
  ArmModel arm({DHParameter()});
  const std::vector<Eigen::Matrix4d> poses = arm.forward_kinematics({0.0});
  assert(poses.size() == 1);
  assert(poses[0].isApprox(Eigen::Matrix4d::Identity(), kTol));
}

void test_pure_rotation() {
  ArmModel arm({DHParameter()});
  const Eigen::Matrix4d t = arm.end_effector_pose({kPi / 2.0});
  assert(close(t(0, 0), 0.0));
  assert(close(t(1, 0), 1.0));
  assert(close(t(0, 1), -1.0));
  assert(close(t(1, 1), 0.0));
}

void test_planar_two_link_matches_trig() {
  const double l1 = 1.0;
  const double l2 = 0.5;
  ArmModel arm({DHParameter(l1), DHParameter(l2)});

  const double theta1 = 30.0 * kPi / 180.0;
  const double theta2 = 45.0 * kPi / 180.0;
  const auto t = arm.end_effector_pose({theta1, theta2});

  const double expected_x = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
  const double expected_y = l1 * std::sin(theta1) + l2 * std::sin(theta1 + theta2);

  assert(close(t(0, 3), expected_x));
  assert(close(t(1, 3), expected_y));
  assert(close(t(2, 3), 0.0));
}

void test_link_twist() {
  // a=1, alpha=90deg, theta=0: origin shifts by `a` along local x, and the
  // y-axis rotates into z (confirms the arm isn't limited to a flat plane).
  ArmModel arm({DHParameter(1.0, kPi / 2.0)});
  const auto t = arm.end_effector_pose({0.0});
  assert(close(t(0, 3), 1.0));
  assert(close(t(1, 1), 0.0));
  assert(close(t(2, 1), 1.0));
}

void test_prismatic_joint() {
  ArmModel arm({DHParameter(0.0, 0.0, 0.0, 0.0, JointType::Prismatic)});
  const Eigen::Matrix4d t = arm.end_effector_pose({2.0});
  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
  expected(2, 3) = 2.0;
  assert(t.isApprox(expected, kTol));
}

} // namespace

int main(void) {
  test_identity_transform();
  test_pure_rotation();
  test_planar_two_link_matches_trig();
  test_link_twist();
  test_prismatic_joint();
  std::cout << "dh_kinematics_tests passed\n";
}
