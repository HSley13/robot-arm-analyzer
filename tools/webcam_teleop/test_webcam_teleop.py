"""Tests for webcam_teleop's control rules, with a fake robot link and fake
tracker results - no camera, MediaPipe or app needed (python3 -m unittest)."""

import unittest
from types import SimpleNamespace

from teleop_math import P
from webcam_teleop import Controller, right_hand


class FakeRobot:
    shoulder = (0.0, 1.25, 0.20)
    reach = 0.665
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
        control.hand([P(0, 0, 0)] * 21, 0.0)
        self.assertEqual(robot.types(), ["hold"])
        control.toggle_pause()
        control.arm(SHOULDER, ELBOW, WRIST, 0.1)
        self.assertEqual(robot.types(), ["hold", "reach"])

    def test_a_hand_sends_its_grip(self):
        robot = FakeRobot()
        hand = [P(0, 0, 0) for _ in range(21)]
        hand[9] = P(0, -0.09, 0)
        for tip in (8, 12, 16, 20):
            hand[tip] = P(0, -0.09, 0)  # fingertips folded back to the knuckles: a fist
        Controller(robot, mirror=False).hand(hand, 0.0)
        self.assertEqual(robot.types(), ["grip"])
        self.assertEqual(robot.sent[0]["amount"], 1.0)


class RightHand(unittest.TestCase):
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
