#pragma once

#include <map>
#include <string>
#include <vector>

#include "robotarm/hand_config.hpp"

namespace robotarm {

// Named joint-angle TARGET poses for a hand, generic over its actual
// chain structure - a different HandConfig (more/fewer fingers, a
// different wrist) still works, nothing here hardcodes a specific chain
// name or joint count. Every pose is keyed by chain name with one value
// (radians) per joint, matching KinematicTree::forward_kinematics()'s
// input shape directly.
//
// This class only produces TARGETS, never animates - it has no notion of
// "the hand's current pose," so it needs no QObject/timer machinery and
// is plain GTest-testable. Smoothly moving toward a target is the
// caller's job (HandController), via CubicTrajectory/RecordedTrajectory.
class HandGestures {
public:
  explicit HandGestures(HandConfig config) : _config(std::move(config)) {}

  // Every joint at exactly 0 - the configuration a hand's own fixed
  // theta_offset_deg values (e.g. each finger's splay) were designed
  // around, so this is "mirror a relaxed real hand with nothing in it,"
  // not an arbitrary zero.
  std::map<std::string, std::vector<double>> restPose() const;

  // Every real (non-fixed-placement) joint driven to its own
  // min_degrees - a flat, splayed-open palm. Excludes the root chain
  // (the one with an empty parent_chain, e.g. "wrist") entirely: opening
  // the hand shouldn't also reset wherever the wrist currently is.
  std::map<std::string, std::vector<double>> openPalm() const;

  // Every real finger joint driven to its own max_degrees - a closed
  // fist. Same root-chain exclusion as openPalm().
  std::map<std::string, std::vector<double>> closedFist() const;

  // Fingers wrapped around a handle (a dumbbell bar, a bag strap) rather
  // than clenched shut - every real finger joint at kPowerGripFraction of
  // the way from its min_degrees to its max_degrees. Same root-chain
  // exclusion as closedFist(). This is the pose the hand takes while it's
  // holding the arm's payload.
  std::map<std::string, std::vector<double>> powerGrip() const;
  static constexpr double kPowerGripFraction = 0.7;

  // Fingers closed `fraction` of the way (0 = open palm, 1 = fist) - how
  // far depends on what's being held: a thin handle needs ~0.7, a ball
  // ~0.35. Same root-chain exclusion as the other poses; fraction is
  // clamped to [0, 1].
  std::map<std::string, std::vector<double>> graspAtFraction(double fraction) const;

  // A short back-and-forth swing of the root chain's first joint (by
  // convention the one closest to "forearm roll") - "wave hello."
  // Returns the sequence of TARGET waypoints only (not including
  // whatever pose the hand happens to be in right now, which the caller
  // already knows and this class deliberately doesn't). If the root
  // chain has no joints at all, there is nothing to wave, and this
  // degenerates to a single no-op waypoint at rest rather than an
  // out-of-bounds access.
  std::vector<std::map<std::string, std::vector<double>>> wave() const;

private:
  // fraction=0 -> every real joint at its own min_degrees; fraction=1 ->
  // every real joint at its own max_degrees. A "fixed placement" joint
  // (min_degrees == max_degrees) is always left at 0 regardless of
  // fraction, and the root chain is included only if requested.
  std::map<std::string, std::vector<double>> poseAtFraction(double fraction, bool includeRootChain) const;

  const HandChainConfig *findChain(const std::string &name) const;
  const HandChainConfig *rootChain() const;

  HandConfig _config;
};

} // namespace robotarm
