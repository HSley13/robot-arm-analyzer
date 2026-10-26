// Smoke test for Module 1: proves Eigen, nlohmann::json, and the
// robotarm_core library all configure, link, and run correctly end to end
// before any real kinematics/statics code lands in later modules.
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "robotarm/version.hpp"

TEST(Scaffolding, EigenIdentity) {
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  EXPECT_EQ(identity(0, 0), 1.0);
  EXPECT_EQ(identity(1, 1), 1.0);
  EXPECT_EQ(identity(0, 1), 0.0);
}

TEST(Scaffolding, JsonParse) {
  const nlohmann::json config = nlohmann::json::parse(R"({"joints": 3})");
  EXPECT_EQ(config.at("joints").get<int>(), 3);
}

TEST(Scaffolding, CoreVersion) {
  EXPECT_EQ(robotarm::core_version(), robotarm::kCoreVersion);
}
