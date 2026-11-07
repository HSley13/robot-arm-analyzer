#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "robotarm/arm_config.hpp"
#include "robotarm/arm_model.hpp"
#include "robotarm/pick_planner.hpp"
#include "robotarm/units.hpp"

using robotarm::ArmConfig;
using robotarm::ArmModel;
using robotarm::BodyGeometry;
using robotarm::GraspTarget;
using robotarm::PickPlan;
using robotarm::kPi;

namespace {

ArmConfig humanoid_arm() {
  return robotarm::load_arm_config(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json");
}

// Where the hand's grip center actually ends up for a plan: forward
// kinematics in the arm plane, then the plane placed in the world by the
// body pose - the same chain of transforms the 3D view applies.
Eigen::Vector3d grip_point_of(const ArmConfig &config, const BodyGeometry &body, const PickPlan &plan,
                              const GraspTarget &target) {
  const ArmModel arm = robotarm::to_arm_model(config);
  const Eigen::Matrix4d tip = arm.forward_kinematics(plan.pose.arm).back();
  // Arm frame -> the (unturned) body plane: rotate by -bend, then offset
  // by the shoulder's position.
  const Eigen::Vector2d tip_plane = Eigen::Rotation2Dd(-plan.pose.hip_bend) * Eigen::Vector2d(tip(0, 3), tip(1, 3)) +
                                    robotarm::shoulder_in_plane(body, plan.pose.hip_height, plan.pose.hip_bend);
  const double phi = plan.hand_angle;
  // In the torso frame: x forward, y up, z to the right; the object center
  // is forward_offset along the hand and palmar_offset toward -z from the
  // arm's plane.
  const Eigen::Vector3d local(tip_plane.x() + target.forward_offset * std::cos(phi),
                              tip_plane.y() + target.forward_offset * std::sin(phi),
                              body.shoulder_lateral - target.palmar_offset);
  const double c = std::cos(plan.pose.yaw);
  const double s = std::sin(plan.pose.yaw);
  return {local.x() * c + local.z() * s, local.y(), -local.x() * s + local.z() * c};
}

} // namespace

TEST(PlanarIk, BothElbowSolutionsReachTheTargetWithTheRequestedHandAngle) {
  const ArmModel arm({robotarm::DHParameter(0.30), robotarm::DHParameter(0.27), robotarm::DHParameter(0.095)});
  const Eigen::Vector2d target(0.35, -0.25);
  const double phi = robotarm::to_radians(-20.0);

  const auto solutions = robotarm::planar_3r_ik(0.30, 0.27, 0.095, target, phi);
  ASSERT_EQ(solutions.size(), 2u);
  for (const auto &q : solutions) {
    const Eigen::Matrix4d tip = arm.forward_kinematics(q).back();
    EXPECT_NEAR(tip(0, 3), target.x(), 1e-9);
    EXPECT_NEAR(tip(1, 3), target.y(), 1e-9);
    EXPECT_NEAR(q[0] + q[1] + q[2], phi, 1e-9); // planar: the hand's angle is the sum
  }
  // The two solutions bend the elbow opposite ways.
  EXPECT_LT(solutions[0][1] * solutions[1][1], 0.0);
}

TEST(PlanarIk, NoSolutionBeyondReach) {
  EXPECT_TRUE(robotarm::planar_3r_ik(0.30, 0.27, 0.095, Eigen::Vector2d(0.9, 0.0), 0.0).empty());
}

TEST(PlanarIk, ThetaOffsetsAreSubtractedOut) {
  const std::vector<double> offsets = {0.1, -0.2, 0.3};
  const auto plain = robotarm::planar_3r_ik(0.3, 0.27, 0.095, Eigen::Vector2d(0.3, 0.1), 0.2);
  const auto shifted = robotarm::planar_3r_ik(0.3, 0.27, 0.095, Eigen::Vector2d(0.3, 0.1), 0.2, offsets);
  ASSERT_EQ(plain.size(), shifted.size());
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(shifted[0][i], plain[0][i] - offsets[i], 1e-9);
  }
}

TEST(WaistYaw, PutsThePointAtTheRequestedPlaneOffset) {
  for (const Eigen::Vector3d point : {Eigen::Vector3d(0.5, 0, 0.0), Eigen::Vector3d(0.1, 0, 0.6),
                                      Eigen::Vector3d(-0.3, 0, -0.4), Eigen::Vector3d(0.4, 0, -0.2)}) {
    double forward = 0.0;
    const auto yaw = robotarm::yaw_to_place_point_in_plane(point, 0.16, forward);
    ASSERT_TRUE(yaw.has_value());
    // Back into the torso frame: rotate the world point by -yaw.
    const double c = std::cos(*yaw);
    const double s = std::sin(*yaw);
    const double local_x = point.x() * c - point.z() * s;
    const double local_z = point.x() * s + point.z() * c;
    EXPECT_NEAR(local_x, forward, 1e-9);
    EXPECT_NEAR(local_z, 0.16, 1e-9);
  }
}

TEST(WaistYaw, NoAnswerInsideTheBody) {
  double forward = 0.0;
  EXPECT_FALSE(robotarm::yaw_to_place_point_in_plane(Eigen::Vector3d(0.05, 0, 0.05), 0.16, forward).has_value());
}

TEST(PickPlanner, GraspsABottleOnTheFloorByCrouchingAndBending) {
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  GraspTarget bottle;
  bottle.grip_point = Eigen::Vector3d(0.55, 0.09, 0.25);
  bottle.palmar_offset = 0.06;
  bottle.forward_offset = 0.007;
  bottle.mass = 1.0;

  const PickPlan plan = robotarm::plan_grasp(config, body, bottle, 0.8);
  ASSERT_TRUE(plan.ok) << plan.reason;
  // Had to get the shoulder down near the floor: lower the lift, lean in.
  EXPECT_LT(robotarm::shoulder_in_plane(body, plan.pose.hip_height, plan.pose.hip_bend).y(), 0.9);
  EXPECT_GT(plan.pose.hip_bend, 0.0);
  EXPECT_NEAR(plan.hand_angle, 0.0, 1e-12);   // upright object -> level hand
  EXPECT_LT(plan.peak_load_ratio, 1.0);
  EXPECT_LT((grip_point_of(config, body, plan, bottle) - bottle.grip_point).norm(), 1e-6);

  // Every joint within its travel.
  for (size_t i = 0; i < 3; i++) {
    const double degrees = robotarm::to_degrees(plan.pose.arm[i]);
    EXPECT_GE(degrees, config.joints[i].min_degrees - 1e-6);
    EXPECT_LE(degrees, config.joints[i].max_degrees + 1e-6);
  }
}

TEST(PickPlanner, TurnsTheWaistTowardAnObjectToTheSide) {
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  GraspTarget target;
  target.grip_point = Eigen::Vector3d(0.05, 0.85, 0.60); // off to the robot's right, table height
  target.palmar_offset = 0.06;
  target.mass = 0.5;

  const PickPlan plan = robotarm::plan_grasp(config, body, target, 0.8);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_LT(plan.pose.yaw, -0.5); // turned right (negative yaw about +Y)
  EXPECT_LT((grip_point_of(config, body, plan, target) - target.grip_point).norm(), 1e-6);
}

TEST(PickPlanner, ReportsOutOfReach) {
  GraspTarget far;
  far.grip_point = Eigen::Vector3d(1.8, 0.5, 0.2);
  const PickPlan plan = robotarm::plan_grasp(humanoid_arm(), BodyGeometry(), far, 0.8);
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.reason, "out of reach");
}

TEST(PickPlanner, ReportsTooCloseToTheBody) {
  GraspTarget near;
  near.grip_point = Eigen::Vector3d(0.05, 0.8, 0.10);
  const PickPlan plan = robotarm::plan_grasp(humanoid_arm(), BodyGeometry(), near, 0.8);
  EXPECT_FALSE(plan.ok);
  EXPECT_NE(plan.reason.find("too close"), std::string::npos);
}

TEST(PickPlanner, RefusesAnObjectTooHeavyForTheJointsAndSaysWhichJoint) {
  GraspTarget heavy;
  heavy.grip_point = Eigen::Vector3d(0.55, 0.10, 0.2);
  heavy.palmar_offset = 0.04;
  heavy.mass = 60.0;
  const PickPlan plan = robotarm::plan_grasp(humanoid_arm(), BodyGeometry(), heavy, 0.8);
  EXPECT_FALSE(plan.ok);
  EXPECT_NE(plan.reason.find("too heavy"), std::string::npos) << plan.reason;
  EXPECT_GT(plan.peak_load_ratio, 1.0);
}

TEST(PickPlanner, MayTiltTheHandForObjectsThatDontNeedToStayUpright) {
  // High and close in (a top shelf right in front of the face): a level
  // hand can't fold in there within the joints' travel, a tilted one can.
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  GraspTarget ball;
  ball.grip_point = Eigen::Vector3d(0.34, 1.50, 0.25);
  ball.palmar_offset = 0.075;
  ball.mass = 0.4;

  ball.keep_hand_level = true;
  const PickPlan level = robotarm::plan_grasp(config, body, ball, 0.8);
  ball.keep_hand_level = false;
  const PickPlan tilted = robotarm::plan_grasp(config, body, ball, 0.8);

  EXPECT_FALSE(level.ok);
  ASSERT_TRUE(tilted.ok) << tilted.reason;
  EXPECT_NE(tilted.hand_angle, 0.0);
  EXPECT_LT((grip_point_of(config, body, tilted, ball) - ball.grip_point).norm(), 1e-6);
}

TEST(PickPlanner, RejectsArmsItCantPlanFor) {
  ArmConfig config = humanoid_arm();
  config.joints[1].dh.alpha = 1.0; // out of plane
  const PickPlan plan = robotarm::plan_grasp(config, BodyGeometry(), GraspTarget(), 1.0);
  EXPECT_FALSE(plan.ok);
}

namespace {
// A five-joint warehouse-style arm with wide joint travel.
ArmConfig five_joint_arm() {
  ArmConfig config = humanoid_arm();
  config.joints.clear();
  const double lengths[] = {0.22, 0.20, 0.16, 0.12, 0.08};
  for (const double a : lengths) {
    robotarm::JointConfig joint;
    joint.dh = robotarm::DHParameter(a);
    joint.min_degrees = -170.0;
    joint.max_degrees = 170.0;
    joint.mass = 0.8;
    joint.torque_limit = 80.0;
    config.joints.push_back(joint);
  }
  return config;
}
} // namespace

TEST(PickPlanner, PlansForArmsWithMoreJointsToo) {
  const ArmConfig config = five_joint_arm();
  const BodyGeometry body;
  GraspTarget box;
  box.grip_point = Eigen::Vector3d(0.60, 0.20, 0.20);
  box.palmar_offset = 0.0;
  box.forward_offset = 0.06;
  box.mass = 1.0;
  box.hand_angles = {-kPi / 2.0}; // gripper straight down, from above

  const PickPlan plan = robotarm::plan_grasp(config, body, box, 0.8);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_EQ(plan.pose.arm.size(), 5u);
  EXPECT_NEAR(plan.hand_angle, -kPi / 2.0, 1e-12);
  EXPECT_LT((grip_point_of(config, body, plan, box) - box.grip_point).norm(), 1e-3);
}

TEST(PickPlanner, ExplicitHandAnglesOverrideTheLevelRule) {
  GraspTarget can;
  can.grip_point = Eigen::Vector3d(0.55, 0.20, 0.25);
  can.palmar_offset = 0.04;
  can.mass = 0.3;
  can.keep_hand_level = true;
  can.hand_angles = {-kPi / 4.0};
  const PickPlan plan = robotarm::plan_grasp(humanoid_arm(), BodyGeometry(), can, 0.8);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_NEAR(plan.hand_angle, -kPi / 4.0, 1e-12);
}

namespace {
robotarm::BodyPose standing_pose() {
  robotarm::BodyPose pose;
  pose.yaw = 0.0;
  pose.hip_height = 0.8;
  pose.hip_bend = 0.0;
  pose.arm = {robotarm::to_radians(-70.0), robotarm::to_radians(65.0), robotarm::to_radians(5.0)};
  return pose;
}

Eigen::Vector3d tip_point_of(const ArmConfig &config, const BodyGeometry &body, const robotarm::BodyPose &pose) {
  const ArmModel arm = robotarm::to_arm_model(config);
  const Eigen::Matrix4d tip = arm.forward_kinematics(pose.arm).back();
  const Eigen::Vector2d plane = Eigen::Rotation2Dd(-pose.hip_bend) * Eigen::Vector2d(tip(0, 3), tip(1, 3)) +
                                robotarm::shoulder_in_plane(body, pose.hip_height, pose.hip_bend);
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {plane.x() * c + body.shoulder_lateral * s, plane.y(), -plane.x() * s + body.shoulder_lateral * c};
}
} // namespace

TEST(ReachPlanner, FollowsANearbyPointWithTheArmAlone) {
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  const robotarm::BodyPose start = standing_pose();
  const Eigen::Vector3d point(0.40, 1.05, 0.30);
  const PickPlan plan = robotarm::plan_reach(config, body, point, start);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_DOUBLE_EQ(plan.pose.hip_height, start.hip_height); // body didn't have to move
  EXPECT_DOUBLE_EQ(plan.pose.hip_bend, start.hip_bend);
  EXPECT_LT((tip_point_of(config, body, plan.pose) - point).norm(), 1e-3);
}

TEST(ReachPlanner, CrouchesAndLeansForAPointNearTheFloor) {
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  const Eigen::Vector3d point(0.55, 0.10, -0.20);
  const PickPlan plan = robotarm::plan_reach(config, body, point, standing_pose());
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_LT((tip_point_of(config, body, plan.pose) - point).norm(), 1e-3);
}

TEST(ReachPlanner, StopsShortButStillGivesAPoseWhenOutOfReach) {
  const ArmConfig config = humanoid_arm();
  const PickPlan plan = robotarm::plan_reach(config, BodyGeometry(), Eigen::Vector3d(2.5, 1.0, 0.0), standing_pose());
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.pose.arm.size(), 3u);
}

TEST(ReachPlanner, OutOfReachPosturesDontFlipAsTheTargetCreeps) {
  // Dragging the hand beyond reach: each small step of the target should
  // only nudge the pose, never swap to a different way of leaning.
  const ArmConfig config = humanoid_arm();
  const BodyGeometry body;
  robotarm::BodyPose pose = standing_pose();
  for (int i = 0; i < 20; i++) {
    const Eigen::Vector3d point(1.3 + 0.01 * i, 0.4 + 0.01 * i, 0.2);
    const PickPlan plan = robotarm::plan_reach(config, body, point, pose);
    ASSERT_FALSE(plan.ok);
    if (i > 0) {
      EXPECT_LT(std::abs(plan.pose.hip_height - pose.hip_height), 0.11) << "step " << i;
      EXPECT_LT(std::abs(plan.pose.hip_bend - pose.hip_bend), robotarm::to_radians(16.0)) << "step " << i;
    }
    pose = plan.pose;
  }
}

TEST(ReachPlanner, WorksForAFiveJointArm) {
  const ArmConfig config = five_joint_arm();
  robotarm::BodyPose start = standing_pose();
  start.arm = {-0.6, 1.0, -0.8, 0.6, 0.2};
  const Eigen::Vector3d point(0.50, 0.90, -0.30);
  const PickPlan plan = robotarm::plan_reach(config, BodyGeometry(), point, start);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_LT((tip_point_of(config, BodyGeometry(), plan.pose) - point).norm(), 1e-3);
}
