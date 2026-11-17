#include <gtest/gtest.h>

#include "robotarm/intent.hpp"

using robotarm::assist_strength;
using robotarm::IntentEstimator;

namespace {

constexpr double kFrame = 1.0 / 30.0;

// Moves the hand from `from` toward `to` at `speed` m/s for `seconds`,
// telling the estimator every frame. Returns where the hand ended up.
Eigen::Vector3d move(IntentEstimator &estimator, Eigen::Vector3d from, const Eigen::Vector3d &to, double speed,
                     double seconds, const std::vector<IntentEstimator::Candidate> &candidates) {
  const Eigen::Vector3d step = (to - from).normalized() * speed * kFrame;
  for (double t = 0.0; t < seconds; t += kFrame) {
    from += step;
    estimator.update(from, kFrame, candidates);
  }
  return from;
}

void hold(IntentEstimator &estimator, const Eigen::Vector3d &at, double seconds,
          const std::vector<IntentEstimator::Candidate> &candidates) {
  for (double t = 0.0; t < seconds; t += kFrame) {
    estimator.update(at, kFrame, candidates);
  }
}

const Eigen::Vector3d kStart(0.30, 1.0, 0.25);
const std::vector<IntentEstimator::Candidate> kTwoObjects = {{1, Eigen::Vector3d(0.70, 0.85, 0.05)},
                                                            {2, Eigen::Vector3d(0.70, 0.85, 0.55)}};

} // namespace

TEST(Intent, HasNoGuessBeforeTheHandDoesAnything) {
  IntentEstimator estimator;
  hold(estimator, kStart, 1.0, kTwoObjects);
  EXPECT_EQ(estimator.likely().id, -1);
  EXPECT_GT(estimator.free_belief(), 0.8);
}

TEST(Intent, PicksTheObjectTheHandIsHeadingFor) {
  IntentEstimator estimator;
  move(estimator, kStart, kTwoObjects[1].point, 0.3, 0.5, kTwoObjects);
  EXPECT_EQ(estimator.likely().id, 2);
  EXPECT_GT(estimator.belief(2), 2.0 * estimator.belief(1));
}

TEST(Intent, ChangesItsMindMidGesture) {
  IntentEstimator estimator;
  const Eigen::Vector3d midway = move(estimator, kStart, kTwoObjects[1].point, 0.3, 0.4, kTwoObjects);
  ASSERT_EQ(estimator.likely().id, 2);
  move(estimator, midway, kTwoObjects[0].point, 0.3, 0.4, kTwoObjects);
  EXPECT_EQ(estimator.likely().id, 1);
}

TEST(Intent, GrowsSureAsTheHandArrivesAndStays) {
  IntentEstimator estimator;
  const Eigen::Vector3d near = move(estimator, kStart, kTwoObjects[0].point, 0.3, 1.4, kTwoObjects);
  const double arriving = estimator.likely().confidence;
  EXPECT_EQ(estimator.likely().id, 1);
  hold(estimator, near, 1.0, kTwoObjects); // stopped right at it, deciding how to grab
  EXPECT_EQ(estimator.likely().id, 1);
  EXPECT_GT(estimator.likely().confidence, 0.6);
  EXPECT_GT(arriving, 0.5);
}

TEST(Intent, LetsGoWhenTheHandMovesAway) {
  IntentEstimator estimator;
  const Eigen::Vector3d near = move(estimator, kStart, kTwoObjects[0].point, 0.3, 0.9, kTwoObjects);
  ASSERT_EQ(estimator.likely().id, 1);
  move(estimator, near, Eigen::Vector3d(0.0, 1.4, 0.05), 0.4, 0.8, kTwoObjects);
  EXPECT_EQ(estimator.likely().id, -1);
}

TEST(Intent, BeliefsAddUpAndFollowTheCandidates) {
  IntentEstimator estimator;
  move(estimator, kStart, kTwoObjects[0].point, 0.3, 0.5, kTwoObjects);
  EXPECT_NEAR(estimator.belief(1) + estimator.belief(2) + estimator.free_belief(), 1.0, 1e-12);
  // Object 1 picked up by hand, say: no longer a candidate.
  estimator.update(kStart, kFrame, {kTwoObjects[1]});
  EXPECT_EQ(estimator.belief(1), 0.0);
  EXPECT_NEAR(estimator.belief(2) + estimator.free_belief(), 1.0, 1e-12);
}

TEST(Intent, ResetForgetsTheGuess) {
  IntentEstimator estimator;
  move(estimator, kStart, kTwoObjects[0].point, 0.3, 0.5, kTwoObjects);
  estimator.reset();
  estimator.update(kStart, kFrame, kTwoObjects);
  EXPECT_EQ(estimator.likely().id, -1);
}

TEST(AssistStrength, NothingUnsureOrFarAway) {
  EXPECT_EQ(assist_strength(0.3, 0.05), 0.0);
  EXPECT_EQ(assist_strength(0.95, 0.40), 0.0);
}

TEST(AssistStrength, FullWhenSureAndClose) {
  EXPECT_DOUBLE_EQ(assist_strength(0.9, 0.05), 0.85);
}

TEST(AssistStrength, FadesInSmoothly) {
  double last = 0.0;
  for (double distance = 0.30; distance >= 0.08; distance -= 0.02) {
    const double strength = assist_strength(0.9, distance);
    EXPECT_GE(strength, last);
    EXPECT_LT(strength - last, 0.15); // no jolt
    last = strength;
  }
  EXPECT_LT(assist_strength(0.6, 0.05), assist_strength(0.8, 0.05));
}
