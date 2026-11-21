#pragma once

#include <vector>

namespace robotarm {

// Joint positions/velocities/accelerations at one instant along a
// trajectory - exactly the three inputs inverse_dynamics() needs.
struct TrajectoryState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
};

// Minimum-jerk easing of a pose-to-pose move: s(u) = 10u^3 - 15u^4 + 6u^5
// for progress u in [0, 1], with its first and second derivatives (per
// unit of u - divide by the duration T, and T^2, for per-second rates).
// Velocity AND acceleration are zero at both ends, so a move starts and
// stops without a jump in acceleration (a cubic, or smoothstep, starts
// with a sudden 6*delta/T^2) - a jump that inverse dynamics would show as
// a torque spike no real joint produces. u outside [0, 1] is clamped,
// at rest.
struct Easing {
  double s = 0.0;   // fraction of the way there
  double ds = 0.0;  // ds/du
  double dds = 0.0; // d^2s/du^2
};
Easing minimum_jerk(double u);

// A smooth pose-to-pose joint trajectory: a quintic polynomial per joint
// (minimum_jerk), at rest - zero velocity and acceleration - at both ends.
// Per the roadmap's "Lessons From Existing Tools" (NovaSolver): stepping
// a joint angle directly - an instant jump from one target to another -
// makes velocity and acceleration spike from zero to infinite/maximum
// instantly, producing torque numbers that are an artifact of the UI, not
// the arm. Use this for any pose-to-pose animation instead.
//
// All joints share the same duration (a "synchronized" trajectory - the
// standard, simplest choice); each joint's own coefficients are still
// independent since start/end differ per joint.
class QuinticTrajectory {
public:
  // `start`/`end` must have the same size (one entry per joint); `duration`
  // must be positive.
  QuinticTrajectory(std::vector<double> start, std::vector<double> end, double duration);

  double duration() const { return _duration; }
  size_t num_joints() const { return _start.size(); }

  // Evaluates the trajectory at time t (seconds, measured from the start
  // of the motion). t < 0 holds at the start pose; t > duration() holds
  // at the end pose - both with zero velocity/acceleration, since
  // extrapolating the quintic beyond its own boundary conditions isn't
  // meaningful.
  TrajectoryState evaluate(double t) const;

private:
  std::vector<double> _start;
  std::vector<double> _end;
  double _duration;
};

} // namespace robotarm
