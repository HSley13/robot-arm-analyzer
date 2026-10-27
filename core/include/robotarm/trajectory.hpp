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

// A smooth pose-to-pose joint trajectory: a cubic polynomial per joint,
// with zero velocity at both endpoints. Per the roadmap's "Lessons From
// Existing Tools" (NovaSolver): stepping a joint angle directly - an
// instant jump from one target to another - makes velocity and
// acceleration spike from zero to infinite/maximum instantly, producing
// torque numbers that are an artifact of the UI, not the arm. Use this
// for any pose-to-pose animation instead.
//
// All joints share the same duration (a "synchronized" trajectory - the
// standard, simplest choice); each joint's own cubic coefficients are
// still independent since start/end differ per joint.
class CubicTrajectory {
public:
  // `start`/`end` must have the same size (one entry per joint); `duration`
  // must be positive.
  CubicTrajectory(std::vector<double> start, std::vector<double> end, double duration);

  double duration() const { return _duration; }
  size_t num_joints() const { return _start.size(); }

  // Evaluates the trajectory at time t (seconds, measured from the start
  // of the motion). t < 0 holds at the start pose; t > duration() holds
  // at the end pose - both with zero velocity/acceleration, since
  // extrapolating the cubic beyond its own boundary conditions isn't
  // meaningful.
  TrajectoryState evaluate(double t) const;

private:
  std::vector<double> _start;
  std::vector<double> _end;
  double _duration;
};

} // namespace robotarm
