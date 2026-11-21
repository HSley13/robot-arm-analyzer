// Verifies QuinticTrajectory against its defining boundary conditions
// (zero velocity and acceleration at both endpoints - the property the
// roadmap requires, to avoid fake torque spikes from a step-change
// profile) and the closed-form peak velocity and acceleration of the
// minimum-jerk profile, plus before/after clamping.
#include <cmath>

#include <gtest/gtest.h>

#include "robotarm/trajectory.hpp"

using robotarm::QuinticTrajectory;
using robotarm::minimum_jerk;
using robotarm::TrajectoryState;

TEST(Trajectory, BoundaryConditionsAreExact) {
  QuinticTrajectory trajectory({0.0, 10.0}, {1.0, -5.0}, 2.0);

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
  // A minimum-jerk trajectory is symmetric
  // about its midpoint - position(T/2) is exactly the average of start
  // and end, independent of T or the endpoints' values.
  QuinticTrajectory trajectory({0.0}, {1.0}, 2.0);
  const TrajectoryState mid = trajectory.evaluate(1.0);
  EXPECT_NEAR(mid.position[0], 0.5, 1e-12);
}

TEST(Trajectory, PeakVelocityMatchesClosedFormResult) {
  // Textbook result for the minimum-jerk profile: the velocity peaks at
  // the midpoint, at 1.875x the average velocity (delta/duration).
  const double start = 0.0;
  const double end = 1.0;
  const double duration = 2.0;
  QuinticTrajectory trajectory({start}, {end}, duration);

  const TrajectoryState mid = trajectory.evaluate(duration / 2.0);
  const double expected_peak_velocity = 1.875 * (end - start) / duration;
  EXPECT_NEAR(mid.velocity[0], expected_peak_velocity, 1e-12);
}

TEST(Trajectory, ClampsBeforeStartAndAfterEnd) {
  QuinticTrajectory trajectory({0.0}, {1.0}, 2.0);

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
  // The profile accelerates away from start then decelerates into
  // end by the same amount - acceleration at t and at (T-t) should be
  // equal in magnitude and opposite in sign.
  QuinticTrajectory trajectory({0.0}, {1.0}, 2.0);
  const double a_early = trajectory.evaluate(0.5).acceleration[0];
  const double a_late = trajectory.evaluate(1.5).acceleration[0];
  EXPECT_NEAR(a_early, -a_late, 1e-12);
}

TEST(Trajectory, NumJointsAndDurationAccessors) {
  QuinticTrajectory trajectory({0.0, 1.0, 2.0}, {1.0, 1.0, 1.0}, 3.5);
  EXPECT_EQ(trajectory.num_joints(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 3.5);
}

TEST(Trajectory, AccelerationStartsAndEndsAtZero) {
  // No jump in acceleration as the move starts or stops: just inside
  // either end it is still close to zero (a cubic would be at 6*delta/T^2).
  QuinticTrajectory trajectory({0.0}, {1.0}, 2.0);
  EXPECT_NEAR(trajectory.evaluate(1e-4).acceleration[0], 0.0, 1e-3);
  EXPECT_NEAR(trajectory.evaluate(2.0 - 1e-4).acceleration[0], 0.0, 1e-3);
}

TEST(Trajectory, PeakAccelerationMatchesClosedFormResult) {
  // Minimum jerk: |acceleration| peaks at u = 1/2 -+ sqrt(3)/6, at
  // (10 / sqrt(3)) * delta / T^2.
  const double delta = 1.0;
  const double duration = 2.0;
  QuinticTrajectory trajectory({0.0}, {delta}, duration);
  const double u_peak = 0.5 - std::sqrt(3.0) / 6.0;
  EXPECT_NEAR(trajectory.evaluate(u_peak * duration).acceleration[0],
              10.0 / std::sqrt(3.0) * delta / (duration * duration), 1e-12);
}

TEST(Trajectory, MinimumJerkDerivativesMatchFiniteDifferences) {
  const double h = 1e-6;
  for (const double u : {0.1, 0.35, 0.5, 0.8}) {
    EXPECT_NEAR(minimum_jerk(u).ds, (minimum_jerk(u + h).s - minimum_jerk(u - h).s) / (2.0 * h), 1e-6);
    EXPECT_NEAR(minimum_jerk(u).dds, (minimum_jerk(u + h).ds - minimum_jerk(u - h).ds) / (2.0 * h), 1e-5);
  }
  EXPECT_DOUBLE_EQ(minimum_jerk(-1.0).s, 0.0);
  EXPECT_DOUBLE_EQ(minimum_jerk(2.0).s, 1.0);
}
