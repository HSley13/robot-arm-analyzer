#include "robotarm/dh.hpp"

#include <cmath>

namespace robotarm {

Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha) {
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  Eigen::Matrix4d t;
  t << ct, -st * ca, st * sa, a * ct,
      st, ct * ca, -ct * sa, a * st,
      0.0, sa, ca, d,
      0.0, 0.0, 0.0, 1.0;
  return t;
}

} // namespace robotarm
