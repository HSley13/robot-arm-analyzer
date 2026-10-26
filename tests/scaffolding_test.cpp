// Smoke test for Module 1: proves Eigen, nlohmann::json, and the
// robotarm_core library all configure, link, and run correctly end to end
// before any real kinematics/statics code lands in later modules.
#include <cassert>
#include <iostream>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "robotarm/version.hpp"

int main() {
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    assert(identity(0, 0) == 1.0 && identity(1, 1) == 1.0);
    assert(identity(0, 1) == 0.0);

    const nlohmann::json config = nlohmann::json::parse(R"({"joints": 3})");
    assert(config.at("joints").get<int>() == 3);

    assert(robotarm::core_version() == robotarm::kCoreVersion);

    std::cout << "scaffolding_test passed (core version "
              << robotarm::core_version() << ")\n";
    return 0;
}
