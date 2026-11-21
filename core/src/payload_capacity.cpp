#include "robotarm/payload_capacity.hpp"

#include <algorithm>
#include <cmath>

namespace robotarm {

namespace {

bool is_within_limits(const Eigen::VectorXd &torque, const std::vector<double> &torque_limits) {
  for (Eigen::Index i = 0; i < torque.size(); i++) {
    if (static_cast<size_t>(i) >= torque_limits.size()) continue;
    const double limit = torque_limits[static_cast<size_t>(i)];
    if (limit <= 0.0) continue; // no limit to check this joint against
    if (std::abs(torque(i)) / limit > 1.0) return false;
  }
  return true;
}

} // namespace

double max_safe_payload(const std::function<Eigen::VectorXd(double)> &torque_for_payload,
                         const std::vector<double> &torque_limits, double max_payload, double tolerance) {
  if (!is_within_limits(torque_for_payload(0.0), torque_limits)) return 0.0;
  if (is_within_limits(torque_for_payload(max_payload), torque_limits)) return max_payload;

  double lo = 0.0;
  double hi = max_payload;
  while (hi - lo > tolerance) {
    const double mid = (lo + hi) / 2.0;
    if (is_within_limits(torque_for_payload(mid), torque_limits)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

double max_safe_payload_linear(const Eigen::VectorXd &torque_at_zero, const Eigen::VectorXd &torque_per_kg,
                               const std::vector<double> &torque_limits, double max_payload) {
  if (!is_within_limits(torque_at_zero, torque_limits)) return 0.0;

  double capacity = max_payload;
  for (Eigen::Index i = 0; i < torque_at_zero.size(); i++) {
    if (static_cast<size_t>(i) >= torque_limits.size()) continue;
    const double limit = torque_limits[static_cast<size_t>(i)];
    const double per_kg = torque_per_kg(i);
    if (limit <= 0.0 || per_kg == 0.0) continue; // unlimited, or the payload doesn't load it
    const double breakeven = (limit - std::copysign(1.0, per_kg) * torque_at_zero(i)) / std::abs(per_kg);
    capacity = std::min(capacity, breakeven);
  }
  return std::max(0.0, capacity);
}

double max_safe_payload_linear(const std::function<Eigen::VectorXd(double)> &torque_for_payload,
                               const std::vector<double> &torque_limits, double max_payload) {
  const Eigen::VectorXd at_zero = torque_for_payload(0.0);
  return max_safe_payload_linear(at_zero, torque_for_payload(1.0) - at_zero, torque_limits, max_payload);
}

} // namespace robotarm
