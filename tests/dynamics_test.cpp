// Verifies LinkInertial::rod() - the documented uniform-slender-rod
// simplification - against the textbook moment-of-inertia formula and
// the geometric placement it depends on (frame_i's local -X pointing
// back toward frame_{i-1} when the link has no twist).
#include <gtest/gtest.h>

#include "robotarm/dynamics.hpp"

using robotarm::LinkInertial;

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
