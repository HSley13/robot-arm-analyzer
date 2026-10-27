// Verifies RecordedTrajectory: boundary conditions across a multi-segment
// recording, that each segment matches a plain CubicTrajectory computed
// independently for that same pair of waypoints (the whole class is just
// a sequencing layer over CubicTrajectory, so this is the real
// correctness check), before/after clamping, and that add_waypoint()
// extends playback.
#include <gtest/gtest.h>

#include "robotarm/recorded_trajectory.hpp"
#include "robotarm/trajectory.hpp"

using robotarm::CubicTrajectory;
using robotarm::RecordedTrajectory;
using robotarm::TrajectoryState;

namespace {
const std::vector<double> kWaypoint0 = {0.0};
const std::vector<double> kWaypoint1 = {1.0};
const std::vector<double> kWaypoint2 = {-0.5};
constexpr double kSegmentDuration = 2.0;
} // namespace

TEST(RecordedTrajectory, TotalDurationIsSegmentCountTimesSegmentDuration) {
  const RecordedTrajectory trajectory({kWaypoint0, kWaypoint1, kWaypoint2}, kSegmentDuration);
  EXPECT_DOUBLE_EQ(trajectory.total_duration(), 2 * kSegmentDuration);
  EXPECT_EQ(trajectory.num_waypoints(), 3u);
}

TEST(RecordedTrajectory, StartsAndEndsAtTheFirstAndLastWaypointAtRest) {
  const RecordedTrajectory trajectory({kWaypoint0, kWaypoint1, kWaypoint2}, kSegmentDuration);

  const TrajectoryState at_start = trajectory.evaluate(0.0);
  EXPECT_DOUBLE_EQ(at_start.position[0], kWaypoint0[0]);
  EXPECT_DOUBLE_EQ(at_start.velocity[0], 0.0);

  const TrajectoryState at_end = trajectory.evaluate(trajectory.total_duration());
  EXPECT_NEAR(at_end.position[0], kWaypoint2[0], 1e-9);
  EXPECT_NEAR(at_end.velocity[0], 0.0, 1e-9);
}

TEST(RecordedTrajectory, StopsAtRestAtTheMiddleWaypointToo) {
  // Each segment is independently a CubicTrajectory with zero velocity
  // at both its own ends, so chaining them means the recording comes to
  // rest at every waypoint, not just the first and last - a real
  // property of this design (independent segments), not a bug, worth
  // asserting explicitly.
  const RecordedTrajectory trajectory({kWaypoint0, kWaypoint1, kWaypoint2}, kSegmentDuration);
  const TrajectoryState at_middle = trajectory.evaluate(kSegmentDuration);
  EXPECT_NEAR(at_middle.position[0], kWaypoint1[0], 1e-9);
  EXPECT_NEAR(at_middle.velocity[0], 0.0, 1e-9);
}

TEST(RecordedTrajectory, EachSegmentMatchesAStandaloneCubicTrajectory) {
  const RecordedTrajectory trajectory({kWaypoint0, kWaypoint1, kWaypoint2}, kSegmentDuration);

  // A time within the second segment (waypoint1 -> waypoint2).
  const double t_in_segment = 0.75; // local time within that segment
  const double t_global = kSegmentDuration + t_in_segment;

  const TrajectoryState from_recording = trajectory.evaluate(t_global);
  const CubicTrajectory standalone_segment(kWaypoint1, kWaypoint2, kSegmentDuration);
  const TrajectoryState from_standalone = standalone_segment.evaluate(t_in_segment);

  EXPECT_NEAR(from_recording.position[0], from_standalone.position[0], 1e-12);
  EXPECT_NEAR(from_recording.velocity[0], from_standalone.velocity[0], 1e-12);
  EXPECT_NEAR(from_recording.acceleration[0], from_standalone.acceleration[0], 1e-12);
}

TEST(RecordedTrajectory, ClampsBeforeStartAndAfterEnd) {
  const RecordedTrajectory trajectory({kWaypoint0, kWaypoint1}, kSegmentDuration);

  const TrajectoryState before = trajectory.evaluate(-5.0);
  EXPECT_DOUBLE_EQ(before.position[0], kWaypoint0[0]);
  EXPECT_DOUBLE_EQ(before.velocity[0], 0.0);

  const TrajectoryState after = trajectory.evaluate(100.0);
  EXPECT_DOUBLE_EQ(after.position[0], kWaypoint1[0]);
  EXPECT_DOUBLE_EQ(after.velocity[0], 0.0);
}

TEST(RecordedTrajectory, AddWaypointExtendsTotalDurationAndPlayback) {
  RecordedTrajectory trajectory({kWaypoint0, kWaypoint1}, kSegmentDuration);
  ASSERT_DOUBLE_EQ(trajectory.total_duration(), kSegmentDuration);

  const std::vector<double> new_waypoint = {2.5};
  trajectory.add_waypoint(new_waypoint);

  EXPECT_EQ(trajectory.num_waypoints(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.total_duration(), 2 * kSegmentDuration);

  const TrajectoryState at_new_end = trajectory.evaluate(trajectory.total_duration());
  EXPECT_NEAR(at_new_end.position[0], new_waypoint[0], 1e-9);
}
