#include "robotarm/trajectory.hpp"

#include <cassert>

namespace robotarm {

Easing minimum_jerk(double u) {
  if (u <= 0.0) return {0.0, 0.0, 0.0};
  if (u >= 1.0) return {1.0, 0.0, 0.0};
  const double u2 = u * u;
  const double u3 = u2 * u;
  return {u3 * (10.0 - 15.0 * u + 6.0 * u2), 30.0 * u2 * (1.0 - u) * (1.0 - u), 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)};
}

QuinticTrajectory::QuinticTrajectory(std::vector<double> start, std::vector<double> end, double duration)
    : _start(std::move(start)), _end(std::move(end)), _duration(duration) {
  assert(_start.size() == _end.size());
  assert(_duration > 0.0);
}

TrajectoryState QuinticTrajectory::evaluate(double t) const {
  const size_t n = _start.size();
  TrajectoryState state;
  state.position.resize(n);
  state.velocity.assign(n, 0.0);
  state.acceleration.assign(n, 0.0);

  if (t <= 0.0) {
    state.position = _start;
    return state;
  }
  if (t >= _duration) {
    state.position = _end;
    return state;
  }

  const Easing easing = minimum_jerk(t / _duration);
  for (size_t i = 0; i < n; i++) {
    const double delta = _end[i] - _start[i];
    state.position[i] = _start[i] + easing.s * delta;
    state.velocity[i] = easing.ds * delta / _duration;
    state.acceleration[i] = easing.dds * delta / (_duration * _duration);
  }

  return state;
}

} // namespace robotarm
