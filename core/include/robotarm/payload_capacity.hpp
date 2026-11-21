#pragma once

#include <functional>
#include <vector>

#include <Eigen/Dense>

namespace robotarm {

// Binary search for the largest `payload` in [0, max_payload] for which
// torque_for_payload(payload) keeps every joint's |torque_i| /
// torque_limits[i] within 1.0. Assumes |torque_i| is monotonically
// non-decreasing in payload over that range - true whenever a heavier
// payload only adds more load (the normal case for a mass hanging from
// the end effector), so the search doesn't need torque to be linear in
// payload, just one-directional.
//
// `torque_for_payload` is a caller-supplied evaluator - the exact
// physics engine, or a trained AI surrogate - so this one search works
// unmodified for either. That's deliberate: the point of this function
// existing at all is to let a caller run the identical algorithm through
// both and compare, not just two numbers that happen to be close on one
// pose.
//
// Returns 0.0 if even payload=0 already exceeds a limit (the pose
// itself is already overloaded, before adding anything), and
// max_payload if the search never finds a violation within that range
// (the arm can hold at least that much - how much more isn't known
// without searching wider).
double max_safe_payload(const std::function<Eigen::VectorXd(double)> &torque_for_payload,
                         const std::vector<double> &torque_limits, double max_payload = 1000.0,
                         double tolerance = 1e-3);

// The same answer in closed form, for torques that are linear in the
// payload: tau(m) = tau(0) + m * per_kg. The exact physics always is - a
// point mass's weight and inertia both scale with its mass, in statics
// and in dynamics alike - so the search above is never needed for it.
// Joint i stays within its limit L_i for every m up to
// (L_i - sign(per_kg_i) * tau_i(0)) / |per_kg_i|; the payload capacity is
// the smallest of those, inside [0, max_payload]. Exact, with no
// tolerance, from two torque evaluations instead of about twenty.
// Same conventions as max_safe_payload: 0 if the pose is already
// overloaded with nothing in the hand, max_payload if no joint binds.
double max_safe_payload_linear(const Eigen::VectorXd &torque_at_zero, const Eigen::VectorXd &torque_per_kg,
                               const std::vector<double> &torque_limits, double max_payload = 1000.0);

// Convenience: evaluates a linear evaluator at 0 and 1 kg and calls the
// closed form above.
double max_safe_payload_linear(const std::function<Eigen::VectorXd(double)> &torque_for_payload,
                               const std::vector<double> &torque_limits, double max_payload = 1000.0);

} // namespace robotarm
