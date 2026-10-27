// Verifies the AI surrogate: it loads its weights and metadata correctly,
// fails loudly on bad input, and - the actual point of the module - its
// predictions land close to the Exact (physics-engine) result on sample
// queries, demonstrating what the plan calls the "genuinely nice demo
// moment": the model is both fast AND accurate.
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/statics.hpp"
#include "robotarm/surrogate.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::LinkMass;
using robotarm::static_joint_torques;
using robotarm::Surrogate;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string weights_path() { return std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar.surrogate.json"; }

// The exact arm the surrogate was trained on (config/two_link_planar.json),
// with the same in-plane gravity override used to generate its training
// data - see Module 5's commit messages for why that override is needed.
ArmModel exact_arm() { return ArmModel({DHParameter(1.0), DHParameter(0.5)}); }
std::vector<LinkMass> exact_links() { return {LinkMass(2.0), LinkMass(1.0)}; }
const Eigen::Vector3d kGravityDownY(0.0, -9.81, 0.0);

// config/three_dof_arm.json (P5-M1) - the shoulder/elbow/wrist arm used by
// the app's default UI from Phase 5 onward.
std::string three_dof_weights_path() {
  return std::string(PROJECT_ROOT_DIR) + "/ml/models/three_dof_arm.surrogate.json";
}
ArmModel three_dof_exact_arm() { return ArmModel({DHParameter(0.9), DHParameter(0.7), DHParameter(0.3)}); }
std::vector<LinkMass> three_dof_exact_links() { return {LinkMass(3.5), LinkMass(2.0), LinkMass(0.8)}; }

} // namespace

TEST(Surrogate, LoadsWeightsAndMetadata) {
  const Surrogate surrogate(weights_path());
  EXPECT_EQ(surrogate.num_joints(), 2u);
  EXPECT_GT(surrogate.val_mean_abs_error(), 0.0);
  EXPECT_LT(surrogate.val_mean_abs_error(), 3.0);
  EXPECT_LT(surrogate.val_max_abs_error(), 15.0);
}

TEST(Surrogate, PredictionsCloseToExactOnSampleQueries) {
  const Surrogate surrogate(weights_path());
  const ArmModel arm = exact_arm();
  const std::vector<LinkMass> links = exact_links();

  // Deterministic sample queries spanning the training ranges (angles in
  // [-pi, pi], payload in [0, 5] kg) - not random, so a failure is always
  // reproducible.
  const std::vector<std::vector<double>> joint_samples = {
      {0.0, 0.0},    {1.0, -1.0},   {-2.0, 2.5},  {3.0, -3.0},
      {-1.5, 0.5},
  };
  const std::vector<double> payload_samples = {0.0, 1.0, 2.5, 4.0, 5.0};

  // A generous bound: Module 5's held-out validation saw 8.14 N*m max
  // error, so allow some margin for a different sample set.
  constexpr double kMaxAllowedError = 15.0;

  for (size_t i = 0; i < joint_samples.size(); i++) {
    const std::vector<double> &joints = joint_samples[i];
    const double payload = payload_samples[i];

    const Eigen::VectorXd exact = static_joint_torques(arm, joints, links, payload, kGravityDownY);
    const std::vector<double> predicted = surrogate.predict(joints, payload);

    ASSERT_EQ(predicted.size(), static_cast<size_t>(exact.size()));
    for (size_t j = 0; j < predicted.size(); j++) {
      EXPECT_NEAR(predicted[j], exact(static_cast<Eigen::Index>(j)), kMaxAllowedError)
          << "joint " << j << " at sample " << i;
    }
  }
}

TEST(Surrogate, ThrowsOnWrongInputSize) {
  const Surrogate surrogate(weights_path());
  EXPECT_THROW(surrogate.predict({0.0, 0.0, 0.0}, 1.0), std::invalid_argument);
  EXPECT_THROW(surrogate.predict({0.0}, 1.0), std::invalid_argument);
}

TEST(Surrogate, ThrowsOnMissingWeightsFile) {
  EXPECT_THROW(Surrogate("/nonexistent/path/does_not_exist.json"), std::runtime_error);
}

TEST(Surrogate, PredictDelegatesToPredictRawIdentically) {
  // predict() is documented as just assembling [joint_values..., payload]
  // and calling predict_raw() - verify that's actually true, not just a
  // comment, so a future refactor of one can't silently diverge from the
  // other.
  const Surrogate surrogate(weights_path());
  ASSERT_EQ(surrogate.input_dim(), surrogate.num_joints() + 1);

  const std::vector<double> joint_values = {1.2, -0.7};
  const double payload = 2.0;

  std::vector<double> raw_input = joint_values;
  raw_input.push_back(payload);

  const std::vector<double> via_predict = surrogate.predict(joint_values, payload);
  const std::vector<double> via_raw = surrogate.predict_raw(raw_input);

  ASSERT_EQ(via_predict.size(), via_raw.size());
  for (size_t i = 0; i < via_predict.size(); i++) {
    EXPECT_DOUBLE_EQ(via_predict[i], via_raw[i]);
  }
}

TEST(Surrogate, PredictRawThrowsOnWrongInputSize) {
  const Surrogate surrogate(weights_path());
  EXPECT_THROW(surrogate.predict_raw({0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(surrogate.predict_raw({0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

TEST(Surrogate, ThreeDofArmLoadsWeightsAndMetadata) {
  const Surrogate surrogate(three_dof_weights_path());
  EXPECT_EQ(surrogate.num_joints(), 3u);
  EXPECT_GT(surrogate.val_mean_abs_error(), 0.0);
  EXPECT_LT(surrogate.val_mean_abs_error(), 5.0);
  EXPECT_LT(surrogate.val_max_abs_error(), 30.0);
}

TEST(Surrogate, ThreeDofArmPredictionsCloseToExactOnSampleQueries) {
  const Surrogate surrogate(three_dof_weights_path());
  const ArmModel arm = three_dof_exact_arm();
  const std::vector<LinkMass> links = three_dof_exact_links();

  // Deterministic sample queries spanning the training ranges (angles in
  // [-pi, pi], payload in [0, 5] kg) - not random, so a failure is always
  // reproducible.
  const std::vector<std::vector<double>> joint_samples = {
      {0.0, 0.0, 0.0},    {1.0, -1.0, 0.5},   {-2.0, 2.5, -1.0},
      {3.0, -3.0, 1.5},   {-1.5, 0.5, -2.5},
  };
  const std::vector<double> payload_samples = {0.0, 1.0, 2.5, 4.0, 5.0};

  // A generous bound: P5-M1's held-out validation saw ~16.2 N*m max
  // error on a joint with a 130 N*m limit, so allow margin for a
  // different sample set.
  constexpr double kMaxAllowedError = 30.0;

  for (size_t i = 0; i < joint_samples.size(); i++) {
    const std::vector<double> &joints = joint_samples[i];
    const double payload = payload_samples[i];

    const Eigen::VectorXd exact = static_joint_torques(arm, joints, links, payload, kGravityDownY);
    const std::vector<double> predicted = surrogate.predict(joints, payload);

    ASSERT_EQ(predicted.size(), static_cast<size_t>(exact.size()));
    for (size_t j = 0; j < predicted.size(); j++) {
      EXPECT_NEAR(predicted[j], exact(static_cast<Eigen::Index>(j)), kMaxAllowedError)
          << "joint " << j << " at sample " << i;
    }
  }
}
