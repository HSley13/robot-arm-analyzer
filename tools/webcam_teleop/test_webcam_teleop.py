"""Tests for webcam_teleop's control rules, with a fake robot link and fake
tracker results - no camera, MediaPipe or app needed (python3 -m unittest)."""

import unittest
from types import SimpleNamespace

import math

from teleop_math import P
from test_teleop_math import curled_hand
from webcam_teleop import Controller, right_hand


class FakeRobot:
    shoulder = (0.0, 1.25, 0.20)
    reach = 0.665
    hand = 0.095
    connected = True

    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(message)

    def types(self):
        return [m["type"] for m in self.sent]


SHOULDER, ELBOW, WRIST = P(0, 0, 0), P(0, 0, -0.3), P(0, 0, -0.6)  # arm straight at the camera


class ControllerRules(unittest.TestCase):
    def test_a_tracked_arm_sends_a_reach_target(self):
        robot = FakeRobot()
        Controller(robot, mirror=False).arm(SHOULDER, ELBOW, WRIST, 0.0)
        self.assertEqual(robot.types(), ["reach"])
        self.assertAlmostEqual(robot.sent[0]["x"], 0.665)  # straight arm: full reach forward
        self.assertAlmostEqual(robot.sent[0]["y"], 1.25)
        self.assertNotIn("pitch", robot.sent[0])  # no hand seen: the planner picks the wrist

    def test_a_seen_hand_tips_the_robots_hand_with_it(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        # Arm straight at the camera, hand tipped 45 degrees up.
        hand = [P(0, 0, 0) for _ in range(21)]
        hand[9] = P(0, -0.06, -0.06)
        control.hand(hand, 0.0)
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        reach = robot.sent[-1]
        self.assertAlmostEqual(reach["pitch"], math.pi / 4)
        # The robot's wrist where yours is, its hand tipped up from there.
        self.assertAlmostEqual(reach["x"], 0.57 + 0.095 * math.cos(math.pi / 4))
        self.assertAlmostEqual(reach["y"], 1.25 + 0.095 * math.sin(math.pi / 4))

    def test_a_hand_not_seen_lately_stops_steering_the_wrist(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        control.hand(curled_hand(0.1, 0.05, 0.05), 0.0)
        control.arm(SHOULDER, ELBOW, WRIST, 1.0)
        self.assertNotIn("pitch", robot.sent[-1])

    def test_no_wrist_leaves_the_wrist_to_the_planner(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False, follow_wrist=False)
        control.hand(curled_hand(0.1, 0.05, 0.05), 0.0)
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        self.assertNotIn("pitch", robot.sent[-1])

    def test_losing_the_person_holds_once(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        control.lost()
        control.lost()  # still gone: nothing more
        self.assertEqual(robot.types(), ["reach", "hold"])

    def test_paused_sends_nothing_but_the_hold(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        control.toggle_pause()
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        control.hand(curled_hand(0.1, 0.05, 0.05), 0.0)
        self.assertEqual(robot.types(), ["hold"])
        control.toggle_pause()
        control.arm(SHOULDER, ELBOW, WRIST, 0.1)
        self.assertEqual(robot.types(), ["hold", "reach"])

    def test_a_hand_sends_its_grip_and_a_held_fist_grabs(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        for i in range(3):
            control.hand(curled_hand(1.5, 1.7, 1.0), i / 30)
        self.assertEqual(robot.types(), ["grip"] * 3)
        self.assertLess(robot.sent[0]["amount"], 0.7)  # one frame isn't enough to grab
        self.assertEqual(robot.sent[-1]["amount"], 1.0)
        self.assertTrue(control.grip_gate.closed)


def label(name):
    return [SimpleNamespace(category_name=name)]


class RightHand(unittest.TestCase):
    def test_a_hand_far_from_the_wrist_isnt_yours(self):
        other = [SimpleNamespace(x=0.8, y=0.5)]
        result = SimpleNamespace(hand_landmarks=[other], hand_world_landmarks=["w"])
        self.assertEqual(right_hand(result, SimpleNamespace(x=0.3, y=0.6)), (None, None))

    def test_the_label_breaks_a_near_tie(self):
        # Both hands at the wrist (hands together): the one labelled "Left" -
        # your right, on an unmirrored frame - wins though a bit farther.
        a = [SimpleNamespace(x=0.30, y=0.60)]
        b = [SimpleNamespace(x=0.33, y=0.60)]
        result = SimpleNamespace(
            hand_landmarks=[a, b], hand_world_landmarks=["a", "b"], handedness=[label("Right"), label("Left")]
        )
        self.assertEqual(right_hand(result, SimpleNamespace(x=0.30, y=0.60))[1], "b")
        self.assertEqual(right_hand(result, SimpleNamespace(x=0.30, y=0.60), mirror=True)[1], "a")

    def test_picks_the_hand_at_the_right_wrist(self):
        left = [SimpleNamespace(x=0.8, y=0.5)]  # far side of the image
        right = [SimpleNamespace(x=0.3, y=0.6)]
        result = SimpleNamespace(hand_landmarks=[left, right], hand_world_landmarks=["left world", "right world"])
        image, world = right_hand(result, SimpleNamespace(x=0.31, y=0.58))
        self.assertIs(image, right)
        self.assertEqual(world, "right world")

    def test_no_hand_or_no_wrist_is_none(self):
        empty = SimpleNamespace(hand_landmarks=[], hand_world_landmarks=[])
        self.assertEqual(right_hand(empty, SimpleNamespace(x=0.3, y=0.6)), (None, None))
        one = SimpleNamespace(hand_landmarks=[[SimpleNamespace(x=0.3, y=0.6)]], hand_world_landmarks=["w"])
        self.assertEqual(right_hand(one, None), (None, None))


if __name__ == "__main__":
    unittest.main()
