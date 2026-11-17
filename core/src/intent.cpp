#include "robotarm/intent.hpp"

#include <algorithm>
#include <cmath>

namespace robotarm {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 0 at edge0, 1 at edge1, smooth in between (either order).
double smoothstep(double edge0, double edge1, double x) {
  const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

} // namespace

void IntentEstimator::update(const Eigen::Vector3d &hand, double seconds, const std::vector<Candidate> &candidates) {
  if (!_started) {
    _started = true;
    _last_hand = hand;
    _velocity.setZero();
    _free_score = _settings.free_evidence;
  } else if (seconds > 0.0) {
    const Eigen::Vector3d raw = (hand - _last_hand) / seconds;
    _velocity += (raw - _velocity) * (1.0 - std::exp(-seconds / _settings.velocity_smoothing));
    _last_hand = hand;
  }

  const double speed = _velocity.norm();
  const double moving = std::min(1.0, speed / _settings.moving_speed);
  const double follow = seconds > 0.0 ? 1.0 - std::exp(-seconds / _settings.memory) : 0.0;

  std::vector<Score> scores;
  scores.reserve(candidates.size());
  for (const Candidate &candidate : candidates) {
    const Eigen::Vector3d to = candidate.point - hand;
    const double distance = to.norm();
    double evidence = -distance / _settings.distance_scale;
    if (speed > 1e-9 && distance > 1e-9) {
      const double angle = std::acos(std::clamp(_velocity.dot(to) / (speed * distance), -1.0, 1.0));
      evidence += _settings.direction_weight * moving * (1.0 - angle / (kPi / 2.0));
    }
    const auto known = std::find_if(_scores.begin(), _scores.end(),
                                    [&](const Score &score) { return score.id == candidate.id; });
    double score = known != _scores.end() ? known->score : _free_score;
    score += (evidence - score) * follow;
    scores.push_back({candidate.id, score});
  }
  _scores = std::move(scores);
  _free_score += (_settings.free_evidence - _free_score) * follow;
}

void IntentEstimator::reset() {
  _scores.clear();
  _started = false;
  _velocity.setZero();
}

std::vector<double> IntentEstimator::beliefs() const {
  double top = _free_score;
  for (const Score &score : _scores) {
    top = std::max(top, score.score);
  }
  std::vector<double> weights;
  double total = 0.0;
  for (const Score &score : _scores) {
    weights.push_back(std::exp(score.score - top));
    total += weights.back();
  }
  weights.push_back(std::exp(_free_score - top));
  total += weights.back();
  for (double &weight : weights) {
    weight /= total;
  }
  return weights;
}

Intent IntentEstimator::likely() const {
  const std::vector<double> weights = beliefs();
  Intent intent;
  for (size_t i = 0; i < _scores.size(); i++) {
    if (weights[i] > intent.confidence && weights[i] > weights.back()) {
      intent = {_scores[i].id, weights[i]};
    }
  }
  return intent;
}

double IntentEstimator::belief(int id) const {
  const std::vector<double> weights = beliefs();
  for (size_t i = 0; i < _scores.size(); i++) {
    if (_scores[i].id == id) {
      return weights[i];
    }
  }
  return 0.0;
}

double IntentEstimator::free_belief() const { return beliefs().back(); }

double assist_strength(double confidence, double distance, const AssistSettings &settings) {
  return settings.max_pull * smoothstep(settings.start_confidence, settings.full_confidence, confidence) *
         smoothstep(settings.far, settings.near, distance);
}

} // namespace robotarm
