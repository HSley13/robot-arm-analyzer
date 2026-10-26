// Verifies the unit-consistency helpers from the roadmap's "Lessons From
// Existing Tools": deg/rad conversion at the boundary, and angle
// normalization so a revolute joint angle never drifts past (-pi, pi] even
// after many wraps.
#include <gtest/gtest.h>

#include "robotarm/units.hpp"

using robotarm::kPi;
using robotarm::normalize_angle;
using robotarm::to_degrees;
using robotarm::to_radians;

TEST(Units, DegreesToRadiansRoundTrip) {
  EXPECT_NEAR(to_radians(180.0), kPi, 1e-12);
  EXPECT_NEAR(to_radians(90.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(to_degrees(to_radians(37.5)), 37.5, 1e-9);
}

TEST(Units, NormalizeAngleIsNoOpInsideRange) {
  EXPECT_NEAR(normalize_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalize_angle(kPi / 4.0), kPi / 4.0, 1e-12);
  EXPECT_NEAR(normalize_angle(-kPi / 4.0), -kPi / 4.0, 1e-12);
}

TEST(Units, NormalizeAngleWrapsPastPi) {
  // 190 degrees should wrap to -170 degrees, not stay at 190.
  const double wrapped = normalize_angle(to_radians(190.0));
  EXPECT_NEAR(to_degrees(wrapped), -170.0, 1e-9);
}

TEST(Units, NormalizeAngleHandlesLargeDrift) {
  // Simulates the exact failure the roadmap calls out: a joint angle that
  // has accumulated many full turns should still normalize to the same
  // physical angle as its un-drifted equivalent.
  const double drifted = to_radians(10.0) + 50.0 * 2.0 * kPi;
  EXPECT_NEAR(to_degrees(normalize_angle(drifted)), 10.0, 1e-6);
}
