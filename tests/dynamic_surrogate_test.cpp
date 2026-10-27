// Verifies the dynamic AI surrogate the same way Module 5 verified the
// static one: it loads correctly, fails loudly on bad input, and - the
// actual point - its predictions land close to exact inverse_dynamics()
// (RNEA) results on sample queries spanning position, velocity,
// acceleration, and payload together.
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "robotarm/arm_model.hpp"
#include "robotarm/dh.hpp"
#include "robotarm/dynamics.hpp"
#include "robotarm/surrogate.hpp"

using robotarm::ArmModel;
using robotarm::DHParameter;
using robotarm::inverse_dynamics;
using robotarm::LinkInertial;
using robotarm::Surrogate;

namespace {

#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

std::string weights_path() {
  return std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar_dynamic.surrogate.json";
}

// The exact arm this surrogate was trained on, with LinkInertial's
// rod() default (no explicit inertia in config/two_link_planar.json) -
// matching exactly what generate_dynamic_training_data.cpp used.
ArmModel exact_arm() { return ArmModel({DHParameter(1.0), DHParameter(0.5)}); }
std::vector<LinkInertial> exact_links() { return {LinkInertial::rod(2.0, 1.0), LinkInertial::rod(1.0, 0.5)}; }
const Eigen::Vector3d kGravityDownY(0.0, -9.81, 0.0);

} // namespace

TEST(DynamicSurrogate, LoadsWeightsAndMetadata) {
  const Surrogate surrogate(weights_path());
  EXPECT_EQ(surrogate.num_joints(), 2u);
  EXPECT_EQ(surrogate.input_dim(), 7u);
  EXPECT_GT(surrogate.val_mean_abs_error(), 0.0);
  EXPECT_LT(surrogate.val_mean_abs_error(), 3.0);
  EXPECT_LT(surrogate.val_max_abs_error(), 20.0);
}

TEST(DynamicSurrogate, PredictionsCloseToExactOnSampleQueries) {
  const Surrogate surrogate(weights_path());
  const ArmModel arm = exact_arm();
  const std::vector<LinkInertial> links = exact_links();

  // Deterministic samples spanning position/velocity/acceleration/payload
  // together - not random, so a failure is reproducible.
  struct Sample {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    double payload;
  };
  const std::vector<Sample> samples = {
      {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 0.0},
      {{1.0, -1.0}, {1.0, -1.0}, {2.0, -2.0}, 1.0},
      {{-2.0, 2.5}, {-1.5, 0.8}, {-3.0, 1.0}, 2.5},
      {{3.0, -3.0}, {0.5, -0.5}, {4.0, -4.0}, 4.0},
      {{-1.5, 0.5}, {1.8, -1.2}, {-1.0, 3.5}, 5.0},
  };

  // A generous bound: the training run's held-out validation saw 12.92
  // N*m max error on a +-160 N*m range, so allow margin for a different
  // sample set.
  constexpr double kMaxAllowedError = 20.0;

  for (size_t i = 0; i < samples.size(); i++) {
    const Sample &s = samples[i];
    const Eigen::VectorXd exact =
        inverse_dynamics(arm, s.position, s.velocity, s.acceleration, links, s.payload, kGravityDownY);

    std::vector<double> raw_input = s.position;
    raw_input.insert(raw_input.end(), s.velocity.begin(), s.velocity.end());
    raw_input.insert(raw_input.end(), s.acceleration.begin(), s.acceleration.end());
    raw_input.push_back(s.payload);
    const std::vector<double> predicted = surrogate.predict_raw(raw_input);

    ASSERT_EQ(predicted.size(), static_cast<size_t>(exact.size()));
    for (size_t j = 0; j < predicted.size(); j++) {
      EXPECT_NEAR(predicted[j], exact(static_cast<Eigen::Index>(j)), kMaxAllowedError)
          << "joint " << j << " at sample " << i;
    }
  }
}

TEST(DynamicSurrogate, PredictRawThrowsOnWrongInputSize) {
  const Surrogate surrogate(weights_path());
  EXPECT_THROW(surrogate.predict_raw({0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(surrogate.predict_raw({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}
