"""Tests for teleop_math (python3 -m unittest, from this folder)."""

import math
import unittest

from teleop_math import FINGERTIPS, MIDDLE_KNUCKLE, WRIST, OneEuroFilter, P, arm_target, grip_amount

ROBOT_SHOULDER = (0.0, 1.25, 0.20)
ROBOT_REACH = 0.665

# A 0.6 m arm: shoulder at the origin, 0.3 m to the elbow, 0.3 m to the wrist.
SHOULDER = P(0.0, 0.0, 0.0)


def arm(wrist: P, elbow: P | None = None) -> tuple:
    elbow = elbow or P(wrist.x / 2, wrist.y / 2, wrist.z / 2)
    return SHOULDER, elbow, wrist


class ArmTarget(unittest.TestCase):
    def test_reaching_toward_the_camera_reaches_forward(self):
        x, y, z = arm_target(*arm(P(0.0, 0.0, -0.6)), ROBOT_SHOULDER, ROBOT_REACH)
        self.assertAlmostEqual(x, ROBOT_REACH)  # a straight arm asks for the full reach
        self.assertAlmostEqual(y, ROBOT_SHOULDER[1])
        self.assertAlmostEqual(z, ROBOT_SHOULDER[2])

    def test_raising_the_hand_raises_the_robots(self):
        _, y, _ = arm_target(*arm(P(0.0, -0.6, 0.0)), ROBOT_SHOULDER, ROBOT_REACH)  # image y points down
        self.assertAlmostEqual(y, ROBOT_SHOULDER[1] + ROBOT_REACH)

    def test_moving_out_to_your_right_moves_the_robot_out_to_its_right(self):
        # Your right is the image's left in an unmirrored webcam frame.
        _, _, z = arm_target(*arm(P(-0.6, 0.0, 0.0)), ROBOT_SHOULDER, ROBOT_REACH)
        self.assertAlmostEqual(z, ROBOT_SHOULDER[2] + ROBOT_REACH)
        _, _, z_mirrored = arm_target(*arm(P(-0.6, 0.0, 0.0)), ROBOT_SHOULDER, ROBOT_REACH, mirror=True)
        self.assertAlmostEqual(z_mirrored, ROBOT_SHOULDER[2] - ROBOT_REACH)

    def test_scales_any_arm_to_the_robots(self):
        # A short arm (0.4 m) stretched out asks for the same full reach.
        short = arm(P(0.0, 0.0, -0.4))
        x, _, _ = arm_target(*short, ROBOT_SHOULDER, ROBOT_REACH)
        self.assertAlmostEqual(x, ROBOT_REACH)
        # Half extended -> half the reach.
        x_half, _, _ = arm_target(SHOULDER, P(0, 0, -0.2), P(0, 0, -0.2), ROBOT_SHOULDER, ROBOT_REACH, arm_length=0.4)
        self.assertAlmostEqual(x_half, ROBOT_REACH / 2)


def hand(fingertip_distance: float, palm: float = 0.09) -> list:
    """21 landmarks: the wrist at the origin, the middle knuckle `palm` up, and
    each fingertip `fingertip_distance` from the wrist."""
    points = [P(0.0, 0.0, 0.0) for _ in range(21)]
    points[MIDDLE_KNUCKLE] = P(0.0, -palm, 0.0)
    for i, tip in enumerate(FINGERTIPS):
        angle = math.radians(-20 + 13 * i)
        points[tip] = P(fingertip_distance * math.sin(angle), -fingertip_distance * math.cos(angle), 0.0)
    points[WRIST] = P(0.0, 0.0, 0.0)
    return points


class GripAmount(unittest.TestCase):
    def test_open_hand_is_zero_and_a_fist_is_one(self):
        self.assertEqual(grip_amount(hand(0.19)), 0.0)  # tips ~2 palms away
        self.assertEqual(grip_amount(hand(0.09)), 1.0)  # tips folded back to ~1 palm

    def test_half_closed_is_in_between(self):
        self.assertAlmostEqual(grip_amount(hand(0.135)), 0.5, places=2)

    def test_the_same_at_any_distance_from_the_camera(self):
        near = grip_amount(hand(0.135, palm=0.09))
        far = grip_amount(hand(0.0675, palm=0.045))  # everything half the size
        self.assertAlmostEqual(near, far)


class Filter(unittest.TestCase):
    def test_smooths_jitter_at_rest(self):
        f = OneEuroFilter()
        values = [f(1.0 + (0.01 if i % 2 else -0.01), i / 30) for i in range(120)]
        self.assertLess(max(values[60:]) - min(values[60:]), 0.01)  # well under the 0.02 raw jitter

    def test_follows_a_fast_move_without_lagging_far_behind(self):
        f = OneEuroFilter()
        for i in range(30):
            f(0.0, i / 30)
        for i in range(30, 45):  # a 0.5 m jump, held for half a second
            out = f(0.5, i / 30)
        self.assertGreater(out, 0.45)

    def test_reset_starts_over(self):
        f = OneEuroFilter()
        f(0.0, 0.0)
        f.reset()
        self.assertEqual(f(5.0, 1.0), 5.0)


if __name__ == "__main__":
    unittest.main()
