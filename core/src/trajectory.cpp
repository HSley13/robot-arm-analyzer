#include "robotarm/trajectory.hpp"

#include <cassert>

namespace robotarm {

CubicTrajectory::CubicTrajectory(std::vector<double> start, std::vector<double> end, double duration)
    : _start(std::move(start)), _end(std::move(end)), _duration(duration) {
  assert(_start.size() == _end.size());
  assert(_duration > 0.0);
}

TrajectoryState CubicTrajectory::evaluate(double t) const {
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

  // Standard cubic polynomial trajectory: q(t) = a0 + a1*t + a2*t^2 +
  // a3*t^3, solved for q(0)=start, q(T)=end, qdot(0)=qdot(T)=0, giving
  // a0=start, a1=0, a2=3*delta/T^2, a3=-2*delta/T^3.
  for (size_t i = 0; i < n; i++) {
    const double delta = _end[i] - _start[i];
    const double a2 = 3.0 * delta / (_duration * _duration);
    const double a3 = -2.0 * delta / (_duration * _duration * _duration);

    state.position[i] = _start[i] + a2 * t * t + a3 * t * t * t;
    state.velocity[i] = 2.0 * a2 * t + 3.0 * a3 * t * t;
    state.acceleration[i] = 2.0 * a2 + 6.0 * a3 * t;
  }

  return state;
}

} // namespace robotarm
