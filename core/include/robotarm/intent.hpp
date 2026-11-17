#pragma once

#include <vector>

#include <Eigen/Dense>

namespace robotarm {

// Guessing what a person steering the robot's hand (with the webcam) is
// reaching for, and how much to help them get there - "shared autonomy":
// the robot only helps as much as it is sure, so it never fights you.
//
// Every update, each candidate object collects evidence from where the
// hand is and where it is heading:
//
//   * closeness:  -distance / distance_scale
//   * heading:    direction_weight * (1 - angle / 90 deg), the angle
//                 between the hand's velocity and the way to the object -
//                 +1 straight at it, 0 sideways, -1 away - counted in full
//                 only when the hand moves at moving_speed or faster (a
//                 still hand has no heading).
//
// Next to them stands "none of these" (free_evidence): just moving the
// arm around, far from everything and toward nothing in particular. Each
// option's score follows its evidence with a memory of `memory` seconds,
// so a guess builds up over a few frames and fades again when the hand
// turns toward something else - mid-gesture. The beliefs are the
// softmax of the scores; they add up to 1 with "none".
struct IntentSettings {
  double distance_scale = 0.15;   // m, evidence -1 per this much distance
  double direction_weight = 5.0;  // evidence for moving straight at it
  double moving_speed = 0.15;     // m/s, from here on the heading counts in full
  double free_evidence = -1.0;    // "none of these"
  double memory = 0.3;            // s, how fast the scores follow the evidence
  double velocity_smoothing = 0.08; // s, the hand's velocity is filtered this much
};

struct Intent {
  int id = -1;             // the likeliest object, -1 = none of them
  double confidence = 0.0; // its belief, 0..1
};

class IntentEstimator {
public:
  struct Candidate {
    int id = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero(); // world, where the hand would grab it
  };

  explicit IntentEstimator(IntentSettings settings = {}) : _settings(settings) {}

  // One observation: the hand (where it would grab, world) `seconds`
  // after the last one, and the objects it could be going for. Objects
  // that are new start with no opinion ("none"'s score); ones that are no
  // longer listed are forgotten.
  void update(const Eigen::Vector3d &hand, double seconds, const std::vector<Candidate> &candidates);
  // Forget everything (the person stepped out of view...).
  void reset();

  Intent likely() const;
  // The belief in one object (0 if it isn't a candidate), and in none.
  double belief(int id) const;
  double free_belief() const;

private:
  struct Score {
    int id;
    double score;
  };
  std::vector<double> beliefs() const; // _scores' order, then "none"

  IntentSettings _settings;
  std::vector<Score> _scores;
  double _free_score = 0.0;
  bool _started = false;
  Eigen::Vector3d _last_hand = Eigen::Vector3d::Zero();
  Eigen::Vector3d _velocity = Eigen::Vector3d::Zero();
};

// How strongly to pull the hand toward the guessed object, 0 (not at all:
// the person steers alone) .. max_pull: nothing until the guess is
// `start_confidence` sure, fully at `full_confidence`; nothing farther
// than `far` from the object, fully from `near` in. Both ramps are smooth
// (smoothstep), so the help fades in and out without a jolt.
struct AssistSettings {
  double start_confidence = 0.45;
  double full_confidence = 0.8;
  double far = 0.30; // m
  double near = 0.08; // m
  double max_pull = 0.85;
};

double assist_strength(double confidence, double distance, const AssistSettings &settings = {});

} // namespace robotarm
