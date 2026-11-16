"""Control the robot arm with your own arm, through the webcam.

Start the robot app first (it listens for outside control on port 8765),
then:

    python3 webcam_teleop.py            # webcam control
    python3 webcam_teleop.py --demo     # no camera: a scripted arm, to try the link

Stand facing the camera, far enough back that it sees your shoulders and
your right arm. Move your right hand and the robot's hand follows - tip
your hand up or down and the robot's tips with it; close your hand near an
object to grab it, open it to let go. Step out of view and the robot holds
still.

Keys in the preview window: space pauses / resumes, q or Esc quits.

How it works, frame by frame:
  webcam -> MediaPipe pose + hand landmarks -> teleop_math (retarget, steady
  heading, hand tilt, grip, smoothing) -> JSON over a WebSocket -> the app's
  SensorBridgeServer,
  which moves the robot with its own planner (smooth, and never past a
  joint's torque limit).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from contextlib import ExitStack
from pathlib import Path

from teleop_math import (
    P,
    GripGate,
    OneEuroFilter,
    arm_target,
    grip_amount,
    hand_direction,
    hand_pitch,
    hand_tip,
    steady_heading,
)

# MediaPipe pose landmark indices: the right arm, as the person sees it.
RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST = 12, 14, 16

MODELS = {
    **{
        f"pose_landmarker_{size}.task": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        f"pose_landmarker_{size}/float16/latest/pose_landmarker_{size}.task"
        for size in ("lite", "full", "heavy")
    },
    "hand_landmarker.task": "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task",
}
MODEL_DIR = Path(__file__).resolve().parent / "models"

# Until the app says otherwise (its hello message): the humanoid's
# standing right shoulder, its arm's reach, and how much of that is the hand.
DEFAULT_SHOULDER = (0.0, 1.25, 0.20)
DEFAULT_REACH = 0.665
DEFAULT_HAND = 0.095

# A hand last seen longer ago than this (s) no longer steers the robot's hand.
HAND_TIMEOUT = 0.3


# ---- the link to the app --------------------------------------------------


class Robot:
    """The WebSocket to the app, reconnecting whenever the app isn't there."""

    def __init__(self, url: str) -> None:
        self.url = url
        self.socket = None
        self._open = ExitStack()  # closes the socket however the program ends
        self.shoulder = DEFAULT_SHOULDER
        self.reach = DEFAULT_REACH
        self.hand = DEFAULT_HAND
        self.kind = "?"
        self._next_try = 0.0

    @property
    def connected(self) -> bool:
        return self.socket is not None

    def ensure_connected(self) -> None:
        """Try to (re)connect, at most every 2 s, without blocking long."""
        if self.socket is not None or time.monotonic() < self._next_try:
            return
        self._next_try = time.monotonic() + 2.0
        from websockets.sync.client import connect

        try:
            self.socket = self._open.enter_context(connect(self.url, open_timeout=1.0))
            # The app introduces the robot first: where its shoulder is and
            # how far its arm reaches, so arms get scaled to this robot.
            hello = json.loads(self.socket.recv(timeout=2.0))
            if hello.get("type") == "hello":
                self.shoulder = tuple(hello["shoulder"])
                self.reach = float(hello["reach"])
                self.hand = float(hello.get("hand", 0.0))  # older apps: no hand, so the wrist is the tip
                self.kind = hello.get("robot", "?")
            print(f"connected: {self.kind} arm, reach {self.reach:.2f} m")
        except Exception as error:  # the app isn't running (yet)
            self._drop()
            print(f"waiting for the robot app at {self.url} ({error.__class__.__name__})", file=sys.stderr)

    def send(self, message: dict) -> None:
        if self.socket is None:
            return
        try:
            self.socket.send(json.dumps(message))
        except Exception:
            self._drop()  # the app closed: reconnect later

    def close(self) -> None:
        if self.socket is not None:
            self.send({"type": "hold"})
        self._drop()

    def _drop(self) -> None:
        self.socket = None
        try:
            self._open.close()
        except Exception:
            pass
        self._open = ExitStack()


# ---- the trackers ---------------------------------------------------------


def model_path(name: str) -> str:
    """The model file, downloaded on first use."""
    path = MODEL_DIR / name
    if not path.exists():
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        print(f"downloading {name} ...")
        urllib.request.urlretrieve(MODELS[name], path)
    return str(path)


def make_trackers(pose_model: str):
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision

    pose = vision.PoseLandmarker.create_from_options(
        vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path(f"pose_landmarker_{pose_model}.task")),
            running_mode=vision.RunningMode.VIDEO,
            min_pose_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
    )
    # Maestro's settings: sure of a hand before taking it (0.7), then keep
    # following it through quick moves (0.5).
    hands = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path("hand_landmarker.task")),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
    )
    return mp, pose, hands


# A hand whose wrist is farther than this from the arm's wrist (fraction of
# the image) is someone else's, or the other hand.
HAND_NEAR_WRIST = 0.15


def right_hand(hand_result, pose_wrist, mirror: bool = False):
    """Which detected hand is the right one: the one at the pose's right
    wrist in the image - preferring, among those, the one MediaPipe labels
    as a right hand. Its labels assume a mirrored selfie image, so on a raw
    webcam frame your right hand is labelled "Left" (as Maestro's hand
    selection has it); the label only breaks ties, position decides."""
    if not hand_result.hand_landmarks or pose_wrist is None:
        return None, None
    expected = "Right" if mirror else "Left"
    labels = getattr(hand_result, "handedness", None) or []

    def score(i):
        wrist = hand_result.hand_landmarks[i][0]
        gap = math.dist((wrist.x, wrist.y), (pose_wrist.x, pose_wrist.y))
        labelled_right = i < len(labels) and labels[i] and labels[i][0].category_name == expected
        return gap - (0.05 if labelled_right else 0.0), gap

    best = min(range(len(hand_result.hand_landmarks)), key=score)
    if score(best)[1] > HAND_NEAR_WRIST:
        return None, None
    return hand_result.hand_landmarks[best], hand_result.hand_world_landmarks[best]


# ---- the main loop --------------------------------------------------------


class Controller:
    """Smoothing, arm-length averaging and the pause / lost-tracking rules.

    Each frame: ``hand()`` first (if a hand is seen) - the grip, and which way
    the hand points - then ``arm()``, which sends the reach, with the hand's
    tilt when a hand was seen lately.
    """

    def __init__(self, robot: Robot, mirror: bool, follow_wrist: bool = True) -> None:
        self.robot = robot
        self.mirror = mirror
        self.follow_wrist = follow_wrist
        # Positions in metres move at up to ~1.5 m/s: beta lets the filter
        # open right up for a quick move, so the robot doesn't trail it.
        self.filters = [OneEuroFilter(min_cutoff=1.0, beta=1.5) for _ in range(3)]
        self.pitch_filter = OneEuroFilter(min_cutoff=1.0, beta=0.5)
        self.grip_filter = OneEuroFilter(min_cutoff=1.5, beta=0.5)
        self.grip_gate = GripGate()
        self.arm_length: float | None = None
        self.heading = (1.0, 0.0)  # (forward, outward): facing straight ahead
        self.paused = False
        self.tracking = False
        self.grip = 0.0
        self.pitch: float | None = None  # the robot hand's tilt, None = left to the planner
        self._hand_direction = None
        self._hand_seen = -math.inf

    def arm(self, shoulder, elbow, wrist, now: float) -> None:
        # Arm length from one frame is noisy (depth guesses); average it slowly.
        length = math.dist((shoulder.x, shoulder.y, shoulder.z), (elbow.x, elbow.y, elbow.z)) + math.dist(
            (elbow.x, elbow.y, elbow.z), (wrist.x, wrist.y, wrist.z)
        )
        self.arm_length = length if self.arm_length is None else 0.97 * self.arm_length + 0.03 * length
        hand = self.robot.hand
        target = arm_target(
            shoulder, elbow, wrist, self.robot.shoulder, self.robot.reach, self.arm_length, self.mirror, hand
        )
        x, y, z = (f(v, now) for f, v in zip(self.filters, target))

        # The waist faces the hand; near the body, it holds its heading.
        sx, _, sz = self.robot.shoulder
        forward, outward, self.heading = steady_heading(x - sx, z - sz, self.heading)
        wrist_target = (sx + forward, y, sz + outward)

        # The robot's hand on its wrist: tipped like yours if your hand was
        # seen lately, else straight on along the arm (as before there was a
        # wrist to steer).
        if self.follow_wrist and self._hand_direction is not None and now - self._hand_seen <= HAND_TIMEOUT:
            raw = hand_pitch(self._hand_direction, self.heading)
            if self.pitch is not None:  # unwrap, so the filter never sees a 2*pi jump
                raw += 2 * math.pi * round((self.pitch - raw) / (2 * math.pi))
            self.pitch = self.pitch_filter(raw, now)
        else:
            self.pitch = None
            self.pitch_filter.reset()
        pitch = self.pitch
        if pitch is None:
            pitch = math.atan2(wrist_target[1] - self.robot.shoulder[1], math.hypot(forward, outward))
        tip = hand_tip(wrist_target, self.heading, pitch, hand)

        self.tracking = True
        if not self.paused:
            message = {"type": "reach", "x": tip[0], "y": tip[1], "z": tip[2]}
            if self.pitch is not None:
                message["pitch"] = math.remainder(self.pitch, 2 * math.pi)
            self.robot.send(message)

    def hand(self, landmarks, now: float) -> None:
        self._hand_direction = hand_direction(landmarks, self.mirror)
        self._hand_seen = now
        self.grip = self.grip_gate(self.grip_filter(grip_amount(landmarks), now))
        if not self.paused:
            self.robot.send({"type": "grip", "amount": self.grip})

    def lost(self) -> None:
        """Out of view: stop where it is (once), and start smoothing afresh."""
        if self.tracking:
            self.robot.send({"type": "hold"})
            for f in self.filters:
                f.reset()
            self.pitch_filter.reset()
            self.pitch = None
        self.tracking = False

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        if self.paused:
            self.robot.send({"type": "hold"})


def run_camera(args, robot: Robot) -> None:
    import cv2

    mp, pose, hands = make_trackers(args.pose_model)
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        sys.exit(f"can't open camera {args.camera} (on macOS, allow the terminal to use the camera)")
    control = Controller(robot, args.mirror, follow_wrist=not args.no_wrist)
    start = time.monotonic()
    stamp = -1

    while True:
        ok, frame = camera.read()
        if not ok:
            break
        robot.ensure_connected()
        now = time.monotonic() - start
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        # VIDEO mode refuses a timestamp that doesn't move forward.
        stamp = max(int(now * 1000), stamp + 1)
        body = pose.detect_for_video(image, stamp)
        found = bool(body.pose_world_landmarks)
        if found:
            world = body.pose_world_landmarks[0]
            seen = body.pose_landmarks[0]
            # Only steer from a clearly visible arm.
            found = min(seen[i].visibility for i in (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)) > 0.5
        if found:
            hand_image, hand_world = right_hand(hands.detect_for_video(image, stamp), seen[RIGHT_WRIST], args.mirror)
            if hand_world is not None:
                control.hand(hand_world, now)
            control.arm(world[RIGHT_SHOULDER], world[RIGHT_ELBOW], world[RIGHT_WRIST], now)
        else:
            control.lost()
            hand_image = None

        if not args.no_preview:
            draw_preview(cv2, frame, seen if found else None, hand_image, control)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                control.toggle_pause()

    camera.release()
    robot.close()


def draw_preview(cv2, frame, pose_landmarks, hand_landmarks, control: Controller) -> None:
    """The camera image as a mirror (what people expect), with the tracked
    arm, the grip, and the state of things."""
    height, width = frame.shape[:2]
    view = cv2.flip(frame, 1)

    def pixel(p):
        return int((1.0 - p.x) * width), int(p.y * height)  # flipped like the view

    if pose_landmarks is not None:
        points = [pixel(pose_landmarks[i]) for i in (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)]
        cv2.line(view, points[0], points[1], (255, 224, 23), 4)
        cv2.line(view, points[1], points[2], (255, 224, 23), 4)
        for p in points:
            cv2.circle(view, p, 7, (255, 255, 255), -1)
    if hand_landmarks is not None:
        for p in hand_landmarks:
            cv2.circle(view, pixel(p), 3, (60, 200, 255), -1)

    # Grip bar: open (left) .. closed (right).
    cv2.rectangle(view, (20, height - 40), (220, height - 20), (80, 80, 80), 2)
    cv2.rectangle(view, (20, height - 40), (20 + int(200 * control.grip), height - 20), (60, 200, 255), -1)
    cv2.line(view, (20 + int(200 * 0.7), height - 44), (20 + int(200 * 0.7), height - 16), (255, 255, 255), 1)
    grip_text = "grip: CLOSED" if control.grip_gate.closed else "grip: open"
    cv2.putText(view, grip_text, (230, height - 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if control.pitch is not None:
        tilt = f"hand tilt {math.degrees(control.pitch):+.0f} deg"
    else:
        tilt = "hand tilt: free (show your hand)" if control.follow_wrist else "hand tilt: off"
    cv2.putText(view, tilt, (20, height - 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if not control.robot.connected:
        status, color = "waiting for the robot app...", (0, 165, 255)
    elif control.paused:
        status, color = "PAUSED - space to resume", (0, 200, 255)
    elif not control.tracking:
        status, color = "step into view: shoulders and right arm", (0, 165, 255)
    else:
        status, color = "controlling the robot", (80, 220, 80)
    cv2.putText(view, status, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.imshow("Robot arm - webcam control (space: pause, q: quit)", view)


def run_demo(robot: Robot, seconds: float) -> None:
    """No camera: a scripted arm sweeping in front of the body and opening
    and closing its hand - to try the link to the app."""
    control = Controller(robot, mirror=False)
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        robot.ensure_connected()
        t = time.monotonic() - start
        # A 0.6 m arm drawing a slow circle in front of the shoulder.
        shoulder = P(0.0, 0.0, 0.0)
        wrist = P(-0.15 * math.cos(t), 0.15 * math.sin(t) + 0.15, -0.45)
        elbow = P(wrist.x / 2, wrist.y / 2 + 0.05, wrist.z / 2)
        control.arm(shoulder, elbow, wrist, t)
        control.grip = 1.0 if math.sin(0.5 * t) > 0 else 0.0
        robot.send({"type": "grip", "amount": control.grip})
        time.sleep(1 / 30)
    robot.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="127.0.0.1", help="computer running the robot app (default: this one)")
    parser.add_argument("--port", type=int, default=8765, help="the app's --bridge-port (default 8765)")
    parser.add_argument("--camera", type=int, default=0, help="camera index (default 0)")
    parser.add_argument("--mirror", action="store_true", help="flip left/right if the robot mirrors you the wrong way")
    parser.add_argument("--no-preview", action="store_true", help="don't show the camera window")
    parser.add_argument("--no-wrist", action="store_true",
                        help="don't copy your hand's tilt: leave the robot's wrist to its planner")
    parser.add_argument("--pose-model", choices=("lite", "full", "heavy"), default="full",
                        help="body tracker: lite is fastest, heavy judges depth best (default full)")
    parser.add_argument("--demo", type=float, nargs="?", const=30.0, metavar="SECONDS",
                        help="no camera: drive the robot with a scripted arm for SECONDS (default 30)")
    args = parser.parse_args()

    robot = Robot(f"ws://{args.host}:{args.port}")
    try:
        if args.demo is not None:
            run_demo(robot, args.demo)
        else:
            run_camera(args, robot)
    except KeyboardInterrupt:
        robot.close()


if __name__ == "__main__":
    main()
