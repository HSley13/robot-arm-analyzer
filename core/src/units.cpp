#include "robotarm/units.hpp"

#include <cmath>

namespace robotarm {

double normalize_angle(double radians) { return std::atan2(std::sin(radians), std::cos(radians)); }

} // namespace robotarm
