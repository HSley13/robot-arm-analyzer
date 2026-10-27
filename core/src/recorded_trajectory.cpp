#include "robotarm/recorded_trajectory.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace robotarm {

RecordedTrajectory::RecordedTrajectory(std::vector<std::vector<double>> waypoints, double segment_duration)
    : _waypoints(std::move(waypoints)), _segmentDuration(segment_duration) {
  assert(_waypoints.size() >= 2);
  assert(_segmentDuration > 0.0);
  for (const std::vector<double> &waypoint : _waypoints) {
    assert(waypoint.size() == _waypoints.front().size());
  }
}

double RecordedTrajectory::total_duration() const {
  return _segmentDuration * static_cast<double>(_waypoints.size() - 1);
}

void RecordedTrajectory::add_waypoint(std::vector<double> pose) {
  assert(pose.size() == _waypoints.front().size());
  _waypoints.push_back(std::move(pose));
}

TrajectoryState RecordedTrajectory::evaluate(double t) const {
  const long num_segments = static_cast<long>(_waypoints.size()) - 1;

  long segment_index = static_cast<long>(std::floor(t / _segmentDuration));
  segment_index = std::max<long>(0, std::min<long>(num_segments - 1, segment_index));

  const double local_t = t - static_cast<double>(segment_index) * _segmentDuration;
  const CubicTrajectory segment(_waypoints[static_cast<size_t>(segment_index)],
                                 _waypoints[static_cast<size_t>(segment_index) + 1], _segmentDuration);
  return segment.evaluate(local_t);
}

} // namespace robotarm
