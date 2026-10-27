// Verifies CubicTrajectory against its defining boundary conditions
// (zero velocity at both endpoints - the exact property the roadmap
// requires, to avoid fake torque spikes from a step-change profile) and
// the well-known closed-form peak-velocity result for this trajectory
// shape, plus before/after clamping.
#include <gtest/gtest.h>

#include "robotarm/trajectory.hpp"

using robotarm::CubicTrajectory;
using robotarm::TrajectoryState;

TEST(Trajectory, BoundaryConditionsAreExact) {
  CubicTrajectory trajectory({0.0, 10.0}, {1.0, -5.0}, 2.0);

  const TrajectoryState at_start = trajectory.evaluate(0.0);
  EXPECT_DOUBLE_EQ(at_start.position[0], 0.0);
  EXPECT_DOUBLE_EQ(at_start.position[1], 10.0);
  EXPECT_DOUBLE_EQ(at_start.velocity[0], 0.0);
  EXPECT_DOUBLE_EQ(at_start.velocity[1], 0.0);

  const TrajectoryState at_end = trajectory.evaluate(2.0);
  EXPECT_NEAR(at_end.position[0], 1.0, 1e-12);
  EXPECT_NEAR(at_end.position[1], -5.0, 1e-12);
  EXPECT_NEAR(at_end.velocity[0], 0.0, 1e-9);
  EXPECT_NEAR(at_end.velocity[1], 0.0, 1e-9);
}

TEST(Trajectory, MidpointIsHalfwayBySymmetry) {
  // A cubic trajectory with zero velocity at both ends is symmetric
  // about its midpoint - position(T/2) is exactly the average of start
  // and end, independent of T or the endpoints' values.
  CubicTrajectory trajectory({0.0}, {1.0}, 2.0);
  const TrajectoryState mid = trajectory.evaluate(1.0);
  EXPECT_NEAR(mid.position[0], 0.5, 1e-12);
}

TEST(Trajectory, PeakVelocityMatchesClosedFormResult) {
  // Textbook result for this exact cubic profile: the velocity peaks at
  // the midpoint, at 1.5x the average velocity (delta/duration).
  const double start = 0.0;
  const double end = 1.0;
  const double duration = 2.0;
  CubicTrajectory trajectory({start}, {end}, duration);

  const TrajectoryState mid = trajectory.evaluate(duration / 2.0);
  const double expected_peak_velocity = 1.5 * (end - start) / duration;
  EXPECT_NEAR(mid.velocity[0], expected_peak_velocity, 1e-12);
}

TEST(Trajectory, ClampsBeforeStartAndAfterEnd) {
  CubicTrajectory trajectory({0.0}, {1.0}, 2.0);

  const TrajectoryState before = trajectory.evaluate(-1.0);
  EXPECT_DOUBLE_EQ(before.position[0], 0.0);
  EXPECT_DOUBLE_EQ(before.velocity[0], 0.0);
  EXPECT_DOUBLE_EQ(before.acceleration[0], 0.0);

  const TrajectoryState after = trajectory.evaluate(5.0);
  EXPECT_DOUBLE_EQ(after.position[0], 1.0);
  EXPECT_DOUBLE_EQ(after.velocity[0], 0.0);
  EXPECT_DOUBLE_EQ(after.acceleration[0], 0.0);
}

TEST(Trajectory, AccelerationIsAntisymmetricAboutTheMidpoint) {
  // The cubic profile accelerates away from start then decelerates into
  // end by the same amount - acceleration at t and at (T-t) should be
  // equal in magnitude and opposite in sign.
  CubicTrajectory trajectory({0.0}, {1.0}, 2.0);
  const double a_early = trajectory.evaluate(0.5).acceleration[0];
  const double a_late = trajectory.evaluate(1.5).acceleration[0];
  EXPECT_NEAR(a_early, -a_late, 1e-12);
}

TEST(Trajectory, NumJointsAndDurationAccessors) {
  CubicTrajectory trajectory({0.0, 1.0, 2.0}, {1.0, 1.0, 1.0}, 3.5);
  EXPECT_EQ(trajectory.num_joints(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 3.5);
}
