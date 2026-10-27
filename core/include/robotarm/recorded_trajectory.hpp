#pragma once

#include <vector>

#include "robotarm/trajectory.hpp"

namespace robotarm {

// A sequence of recorded poses ("waypoints"), played back smoothly
// through consecutive CubicTrajectory segments - the roadmap's Phase 4
// stretch goal: "record/scrub through a sequence of poses over time,
// watching dynamic torque change live... the single best demo of why
// Phase 2's dynamics work mattered." Each segment gets the same
// zero-velocity-at-both-ends smoothness a single CubicTrajectory already
// guarantees, so a multi-point recording is exactly as free of fake
// torque spikes as the original two-point case was - this class is a
// thin sequencing layer over CubicTrajectory, not a new interpolation
// scheme.
class RecordedTrajectory {
public:
  // `waypoints` must have at least 2 entries, all the same size (one
  // value per joint); `segment_duration` applies to every consecutive
  // pair and must be positive.
  RecordedTrajectory(std::vector<std::vector<double>> waypoints, double segment_duration);

  double total_duration() const;
  size_t num_waypoints() const { return _waypoints.size(); }
  const std::vector<std::vector<double>> &waypoints() const { return _waypoints; }

  // Appends a new waypoint at the end - extends total_duration() by one
  // more segment_duration. Must be the same size as the existing ones.
  void add_waypoint(std::vector<double> pose);

  // Evaluates the whole recording at time t (seconds from the start of
  // playback). t < 0 holds at the first waypoint; t > total_duration()
  // holds at the last - both at rest, the same convention
  // CubicTrajectory itself uses at its own boundaries.
  TrajectoryState evaluate(double t) const;

private:
  std::vector<std::vector<double>> _waypoints;
  double _segmentDuration;
};

} // namespace robotarm
