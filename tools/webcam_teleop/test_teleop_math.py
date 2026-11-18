"""Tests for teleop_math (python3 -m unittest, from this folder)."""

import math
import unittest

from teleop_math import (
    FINGERS,
    FINGERTIPS,
    MIDDLE_KNUCKLE,
    WRIST,
    GraspHold,
    GripGate,
    OneEuroFilter,
    P,
    arm_target,
    body_hand_frame,
    finger_curl,
    fingertip_spread,
    grip_amount,
    hand_across,
    hand_crop,
    hand_direction,
    hand_pitch,
    hand_roll,
    hand_tip,
    steady_heading,
)

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

    def test_with_a_hand_your_wrist_goes_where_the_robots_wrist_reaches(self):
        # The robot's hand is added on afterwards (hand_tip): a stretched arm
        # puts the robot's wrist a hand short of its full reach.
        x, _, _ = arm_target(*arm(P(0.0, 0.0, -0.6)), ROBOT_SHOULDER, ROBOT_REACH, robot_hand=0.095)
        self.assertAlmostEqual(x, ROBOT_REACH - 0.095)


class SteadyHeading(unittest.TestCase):
    def test_away_from_the_body_the_heading_follows_the_hand(self):
        forward, outward, heading = steady_heading(0.3, 0.3, (1.0, 0.0))
        self.assertEqual((forward, outward), (0.3, 0.3))
        self.assertAlmostEqual(heading[0], heading[1])  # 45 degrees out

    def test_near_the_body_jitter_cant_turn_the_waist(self):
        heading = (1.0, 0.0)
        for forward, outward in [(0.02, -0.03), (-0.03, 0.01), (0.0, 0.04), (0.01, 0.0)]:
            f, o, heading = steady_heading(forward, outward, heading)
            self.assertEqual(heading, (1.0, 0.0))
            self.assertAlmostEqual(f, 0.12)  # just in front of the body
            self.assertAlmostEqual(o, 0.0)

    def test_coming_back_in_keeps_the_last_heading(self):
        _, _, heading = steady_heading(0.0, 0.4, (1.0, 0.0))  # hand out to the side
        f, o, heading = steady_heading(0.01, 0.02, heading)  # arm drops to the side
        self.assertEqual(heading, (0.0, 1.0))
        self.assertAlmostEqual(o, 0.12)
        self.assertAlmostEqual(f, 0.0)


def flat_hand(dx: float, dy: float, dz: float) -> list:
    """A hand whose wrist-to-middle-knuckle points along (dx, dy, dz) in
    camera coordinates."""
    points = [P(0.0, 0.0, 0.0) for _ in range(21)]
    points[MIDDLE_KNUCKLE] = P(dx, dy, dz)
    return points


class HandTilt(unittest.TestCase):
    def test_a_level_hand_pointing_at_the_camera_is_level(self):
        direction = hand_direction(flat_hand(0.0, 0.0, -0.09))  # toward the camera
        self.assertAlmostEqual(hand_pitch(direction, (1.0, 0.0)), 0.0)

    def test_tipped_up_and_down(self):
        up = hand_direction(flat_hand(0.0, -0.06, -0.06))  # image y points down
        self.assertAlmostEqual(hand_pitch(up, (1.0, 0.0)), math.pi / 4)
        down = hand_direction(flat_hand(0.0, 0.09, 0.0))
        self.assertAlmostEqual(hand_pitch(down, (1.0, 0.0)), -math.pi / 2)

    def test_measured_in_the_plane_the_arm_faces(self):
        # Arm out to your right; the hand points out to the right too: level.
        direction = hand_direction(flat_hand(-0.09, 0.0, 0.0))
        self.assertAlmostEqual(hand_pitch(direction, (0.0, 1.0)), 0.0)

    def test_the_robots_hand_sits_on_its_wrist_tipped_that_way(self):
        x, y, z = hand_tip((0.5, 1.0, 0.2), (1.0, 0.0), math.pi / 2, 0.1)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 1.1)
        self.assertAlmostEqual(z, 0.2)
        x, y, z = hand_tip((0.5, 1.0, 0.2), (0.0, 1.0), 0.0, 0.1)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(z, 0.3)


def turned_hand(thumb_x: float, thumb_y: float, thumb_z: float) -> list:
    """A hand pointing at the camera whose index knuckle sits toward
    (thumb_x, thumb_y, thumb_z) from the pinky knuckle, camera coordinates."""
    points = [P(0.0, 0.0, 0.0) for _ in range(21)]
    points[MIDDLE_KNUCKLE] = P(0.0, 0.0, -0.09)
    points[5] = P(0.04 * thumb_x, 0.04 * thumb_y, -0.09 + 0.04 * thumb_z)
    points[17] = P(-0.04 * thumb_x, -0.04 * thumb_y, -0.09 - 0.04 * thumb_z)
    return points


class HandTwist(unittest.TestCase):
    def roll_of(self, hand, heading=(1.0, 0.0), mirror=False):
        return hand_roll(hand_across(hand, mirror), heading, hand_pitch(hand_direction(hand, mirror), heading))

    def test_thumb_up_is_neutral(self):
        self.assertAlmostEqual(self.roll_of(turned_hand(0, -1, 0)), 0.0)  # image y points down

    def test_palm_up_and_down(self):
        # Your right hand palm up: thumb out to your right - the image's left.
        self.assertAlmostEqual(self.roll_of(turned_hand(-1, 0, 0)), math.pi / 2)
        self.assertAlmostEqual(self.roll_of(turned_hand(1, 0, 0)), -math.pi / 2)

    def test_mirror_flips_it_like_left_and_right(self):
        self.assertAlmostEqual(self.roll_of(turned_hand(-1, 0, 0), mirror=True), -math.pi / 2)

    def test_measured_the_same_with_the_arm_out_to_the_side(self):
        # Arm and hand pointing out to your right (the image's left), palm
        # down: thumb toward the camera.
        points = [P(0.0, 0.0, 0.0) for _ in range(21)]
        points[MIDDLE_KNUCKLE] = P(-0.09, 0.0, 0.0)
        points[5] = P(-0.09, 0.0, -0.04)
        points[17] = P(-0.09, 0.0, 0.04)
        self.assertAlmostEqual(self.roll_of(points, heading=(0.0, 1.0)), -math.pi / 2)

    def test_with_the_hand_tipped_up(self):
        # Fingers straight up, palm toward the camera (a "stop" sign) is the
        # hand tipped up with the palm turned... down, from the robot's view:
        # a level palm-down hand, raised.
        points = [P(0.0, 0.0, 0.0) for _ in range(21)]
        points[MIDDLE_KNUCKLE] = P(0.0, -0.09, 0.0)
        points[5] = P(0.04, -0.09, 0.0)  # thumb side to your left, the image's right
        points[17] = P(-0.04, -0.09, 0.0)
        direction, across = hand_direction(points), hand_across(points)
        pitch = hand_pitch(direction, (1.0, 0.0))
        self.assertAlmostEqual(pitch, math.pi / 2)
        self.assertAlmostEqual(hand_roll(across, (1.0, 0.0), pitch), -math.pi / 2)

    def test_the_body_trackers_hand_points_give_the_same(self):
        direction, across = body_hand_frame(P(0, 0, 0), P(0.0, -0.04, -0.09), P(0.0, 0.04, -0.09))
        pitch = hand_pitch(direction, (1.0, 0.0))
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(hand_roll(across, (1.0, 0.0), pitch), 0.0)  # index above pinky: thumb up


class Crop(unittest.TestCase):
    def test_a_square_around_the_hand(self):
        left, top, side = hand_crop(P(0.5, 0.5, 0), P(0.55, 0.45, 0), P(0.52, 0.42, 0), 640, 480)
        self.assertGreaterEqual(side, 128)
        self.assertLessEqual(left, 0.5 * 640)
        self.assertGreater(left + side, 0.55 * 640)
        self.assertLessEqual(top, 0.42 * 480)

    def test_stays_inside_the_picture(self):
        left, top, side = hand_crop(P(0.99, 0.99, 0), P(0.98, 0.97, 0), P(0.97, 0.98, 0), 640, 480)
        self.assertLessEqual(left + side, 640)
        self.assertLessEqual(top + side, 480)

    def test_a_hand_out_of_the_picture_has_none(self):
        self.assertIsNone(hand_crop(P(1.3, 0.5, 0), P(1.35, 0.5, 0), P(1.32, 0.5, 0), 640, 480))


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


def curled_hand(knuckle: float, middle: float, last: float, palm: float = 0.09, facing: float = 0.0) -> list:
    """A whole hand, fingers bent `knuckle`, `middle` and `last` radians at
    their three joints (0 = straight), curling toward the camera; `facing`
    turns the whole hand about the vertical."""
    points = [P(0.0, 0.0, 0.0) for _ in range(21)]
    bones = (0.045, 0.028, 0.022)
    for i, joints in enumerate(FINGERS):
        splay = math.radians(-12 + 8 * i)
        x, y, z = palm * math.sin(splay), -palm * math.cos(splay), 0.0
        points[joints[0]] = P(x, y, z)
        angle = 0.0
        for joint, bone, bend in zip(joints[1:], bones, (knuckle, middle, last)):
            angle += bend
            x += bone * math.cos(angle) * math.sin(splay)
            y -= bone * math.cos(angle) * math.cos(splay)
            z -= bone * math.sin(angle)
            points[joint] = P(x, y, z)
    c, s = math.cos(facing), math.sin(facing)
    return [P(p.x * c + p.z * s, p.y, -p.x * s + p.z * c) for p in points]


class GripAmount(unittest.TestCase):
    def test_fingertip_spread_open_is_zero_and_a_fist_is_one(self):
        self.assertEqual(fingertip_spread(hand(0.19)), 0.0)  # tips ~2 palms away
        self.assertEqual(fingertip_spread(hand(0.09)), 1.0)  # tips folded back to ~1 palm
        self.assertAlmostEqual(fingertip_spread(hand(0.135)), 0.5, places=2)

    def test_finger_curl_open_is_zero_and_a_fist_is_one(self):
        self.assertEqual(finger_curl(curled_hand(0.1, 0.05, 0.05)), 0.0)
        self.assertEqual(finger_curl(curled_hand(1.5, 1.7, 1.0)), 1.0)
        self.assertAlmostEqual(finger_curl(curled_hand(0.8, 0.8, 0.5)), 0.5, places=2)

    def test_open_hand_is_zero_and_a_fist_is_one(self):
        self.assertEqual(grip_amount(curled_hand(0.1, 0.05, 0.05)), 0.0)
        self.assertAlmostEqual(grip_amount(curled_hand(0.3, 0.3, 0.2)), 0.0)  # relaxed, not quite straight
        self.assertEqual(grip_amount(curled_hand(1.5, 1.7, 1.0)), 1.0)

    def test_the_same_whichever_way_the_hand_faces(self):
        for facing in (0.0, 0.6, 1.2):
            self.assertAlmostEqual(grip_amount(curled_hand(0.8, 0.8, 0.5, facing=facing)), 0.5, places=2)

    def test_the_same_at_any_distance_from_the_camera(self):
        near = grip_amount(curled_hand(0.8, 0.8, 0.5, palm=0.09))
        small = [P(p.x / 2, p.y / 2, p.z / 2) for p in curled_hand(0.8, 0.8, 0.5, palm=0.09)]
        self.assertAlmostEqual(near, grip_amount(small))


class Gate(unittest.TestCase):
    def test_one_glitchy_frame_doesnt_grab(self):
        gate = GripGate()
        outputs = [gate(a) for a in (0.2, 0.9, 0.2, 0.2)]
        self.assertTrue(all(o < 0.7 for o in outputs))
        self.assertFalse(gate.closed)

    def test_a_held_fist_grabs_after_a_few_frames(self):
        gate = GripGate(frames=3)
        outputs = [gate(0.9) for _ in range(3)]
        self.assertEqual(outputs, [0.69, 0.69, 0.9])
        self.assertTrue(gate.closed)

    def test_and_lets_go_the_same_way(self):
        gate = GripGate(frames=3)
        for _ in range(3):
            gate(0.9)
        outputs = [gate(0.1) for _ in range(3)]
        self.assertEqual(outputs, [0.31, 0.31, 0.1])
        self.assertFalse(gate.closed)

    def test_in_between_it_passes_straight_through(self):
        gate = GripGate()
        self.assertEqual(gate(0.5), 0.5)


class Hold(unittest.TestCase):
    def test_an_open_or_closed_hand_steers(self):
        hold = GraspHold()
        self.assertFalse(hold(0.0, 0.05, False))
        hold(1.0, 0.95, True)  # grabbed: settling...
        self.assertFalse(hold(2.0, 0.95, True))  # ...then steering, closed

    def test_the_hand_holds_while_the_fingers_close_and_just_after(self):
        hold = GraspHold(settle=0.25)
        self.assertTrue(hold(0.0, 0.5, False))  # closing
        self.assertTrue(hold(0.1, 0.8, True))  # grabbed: settling
        self.assertTrue(hold(0.3, 0.95, True))
        self.assertFalse(hold(0.4, 0.95, True))  # settled: steering again

    def test_and_while_they_open_again(self):
        hold = GraspHold()
        hold(0.0, 0.95, True)
        hold(0.5, 0.95, True)  # holding something, steering
        self.assertTrue(hold(1.0, 0.5, True))

    def test_never_too_long_a_hand_left_half_closed_steers(self):
        hold = GraspHold(longest=0.8)
        holds = [hold(i / 30, 0.5, False) for i in range(60)]
        self.assertTrue(all(holds[:24]))
        self.assertFalse(any(holds[25:]))


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
