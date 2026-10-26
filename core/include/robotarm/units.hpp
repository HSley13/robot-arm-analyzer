#pragma once

namespace robotarm {

// Single unit convention enforced everywhere internally in this project:
// lengths in meters, angles in radians - DH tables, joint values, physics,
// all of it. Degrees only ever exist at an input/output boundary (a UI
// slider, a JSON config field, a display label) and must be converted via
// these helpers right at that boundary, never carried around half-converted.
constexpr double kPi = 3.14159265358979323846;

constexpr double to_radians(double degrees) { return degrees * kPi / 180.0; }
constexpr double to_degrees(double radians) { return radians * 180.0 / kPi; }

// Wraps an angle in radians to (-pi, pi], the standard revolute-joint range.
// Without this, repeatedly accumulating a joint angle (e.g. integrating a
// velocity over many steps) drifts to arbitrarily large values - normalize
// on every update, not just when the angle is first set.
double normalize_angle(double radians);

} // namespace robotarm
