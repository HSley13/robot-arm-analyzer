"""Tests for webcam_teleop's control rules, with a fake robot link and fake
tracker results - no camera, MediaPipe or app needed (python3 -m unittest)."""

import unittest
from types import SimpleNamespace

import math

from teleop_math import P
import numpy as np

from test_teleop_math import curled_hand, turned_hand
from webcam_teleop import RIGHT_INDEX, RIGHT_PINKY, RIGHT_WRIST, Controller, find_hand, right_hand


class FakeRobot:
    shoulder = (0.0, 1.25, 0.20)
    reach = 0.665
    hand = 0.095
    connected = True
    assist = True

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

    def test_a_turned_palm_twists_the_robots_hand(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        control.hand(turned_hand(-1, 0, 0), 0.0)  # palm up
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        self.assertAlmostEqual(robot.sent[-1]["roll"], math.pi / 2)

    def test_when_the_hand_tracker_loses_it_the_body_tracker_steers(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        # Index above pinky, both ahead of the wrist: level, thumb up.
        control.body_hand(P(0, 0, -0.6), P(0.0, -0.04, -0.69), P(0.0, 0.04, -0.69), 0.0)
        control.arm(SHOULDER, ELBOW, WRIST, 0.0)
        self.assertEqual(robot.types(), ["reach"])  # no grip: it can't tell
        self.assertAlmostEqual(robot.sent[-1]["pitch"], 0.0)
        self.assertAlmostEqual(robot.sent[-1]["roll"], 0.0)
        self.assertEqual(control.hand_source, "body")

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

    def test_the_assist_toggles_off_and_on(self):
        robot = FakeRobot()
        control = Controller(robot, mirror=False)
        control.toggle_assist()
        control.toggle_assist()
        self.assertEqual(robot.sent, [{"type": "assist", "on": False}, {"type": "assist", "on": True}])
        self.assertTrue(robot.assist)

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


class FakeDetector:
    """A hand landmarker that finds `found` (or nothing) and records the
    images it was given."""

    def __init__(self, found=None):
        self.found = found
        self.images = []

    def _result(self, image):
        self.images.append(image)
        if self.found is None:
            return SimpleNamespace(hand_landmarks=[], hand_world_landmarks=[], handedness=[])
        return SimpleNamespace(hand_landmarks=[self.found], hand_world_landmarks=["world"], handedness=[])

    def detect_for_video(self, image, stamp):
        return self._result(image)

    def detect(self, image):
        return self._result(image)


class FakeCv2:
    @staticmethod
    def resize(patch, size):
        return np.zeros((size[1], size[0], 3), dtype=np.uint8)


class FakeMp:
    class ImageFormat:
        SRGB = "srgb"

    @staticmethod
    def Image(image_format, data):
        return data


class FindHand(unittest.TestCase):
    # The body tracker's hand points in a 640x480 frame, near the middle.
    seen = {
        RIGHT_WRIST: SimpleNamespace(x=0.50, y=0.50),
        RIGHT_INDEX: SimpleNamespace(x=0.55, y=0.42),
        RIGHT_PINKY: SimpleNamespace(x=0.52, y=0.42),
    }
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    def test_the_whole_frame_first(self):
        whole = FakeDetector([SimpleNamespace(x=0.5, y=0.5)])
        zoomed = FakeDetector()
        _, world, source = find_hand(FakeMp, FakeCv2, whole, zoomed, self.rgb, "frame", 0, self.seen, False)
        self.assertEqual((world, source), ("world", "hand"))
        self.assertEqual(zoomed.images, [])  # no second look needed

    def test_a_second_look_zoomed_in_maps_back_to_the_frame(self):
        # The zoomed look finds the hand's wrist at the crop's centre.
        zoomed = FakeDetector([SimpleNamespace(x=0.5, y=0.5)])
        image, world, source = find_hand(FakeMp, FakeCv2, FakeDetector(), zoomed, self.rgb, "frame", 0, self.seen, False)
        self.assertEqual((world, source), ("world", "zoomed"))
        self.assertEqual(zoomed.images[0].shape, (256, 256, 3))
        # Back in whole-frame terms: near the body tracker's hand.
        self.assertLess(math.dist((image[0].x, image[0].y), (0.52, 0.45)), 0.05)

    def test_nothing_either_way(self):
        found = find_hand(FakeMp, FakeCv2, FakeDetector(), FakeDetector(), self.rgb, "frame", 0, self.seen, False)
        self.assertEqual(found, (None, None, None))


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
