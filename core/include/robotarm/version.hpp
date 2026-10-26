#pragma once

#include <string_view>

namespace robotarm {

// Bumped manually as Phase 1 modules land (kinematics, statics, config).
constexpr std::string_view kCoreVersion = "0.1.0-phase1";

std::string_view core_version();

}  // namespace robotarm
