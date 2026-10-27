#include "robotarm/payload_capacity.hpp"

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

} // namespace robotarm
