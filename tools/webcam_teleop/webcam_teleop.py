"""Control the robot arm with your own arm, through the webcam.

Start the robot app first (it listens for outside control on port 8765),
then:

    python3 webcam_teleop.py            # webcam control
    python3 webcam_teleop.py --demo     # no camera: a scripted arm, to try the link

Stand facing the camera, far enough back that it sees your shoulders and
arms. Move a hand and the robot's hand on that side follows - tip your hand
up or down, or turn your palm, and the robot's hand does the same; close
your hand near an object to grab it, open it to let go. Both arms work
(if the robot has a working left arm); the hand you moved last steers the
robot's body. Step out of view and the robot holds still.

The app helps: it guesses which object you're reaching for (an amber ring
around it), pulls the robot's hand toward it as it gets sure, and finishes
the grab when you close your hand close by. --no-assist, or the a key,
turns that off.

Keys in the preview window: space pauses / resumes, a turns the assist off /
on, q or Esc quits.

How it works, frame by frame:
  webcam -> MediaPipe pose + hand landmarks -> teleop_math (retarget, steady
  heading, hand tilt, grip, smoothing) -> JSON over a WebSocket -> the app's
  SensorBridgeServer,
  which moves the robot with its own planner (smooth, and never past a
  joint's torque limit).
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
import urllib.request
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from teleop_math import (
    P,
    GraspHold,
    GripGate,
    OneEuroFilter,
    arm_target,
    grip_amount,
    body_hand_frame,
    hand_across,
    hand_crop,
    hand_direction,
    hand_pitch,
    hand_roll,
    hand_tip,
    steady_heading,
)

# MediaPipe pose landmark indices: the right arm, as the person sees it,
# and the body tracker's own rough points on that hand.
RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST = 12, 14, 16
RIGHT_PINKY, RIGHT_INDEX = 18, 20


@dataclass(frozen=True)
class Side:
    """One of your arms: its pose landmarks, and what MediaPipe's hand
    tracker calls that hand on an unmirrored frame (it assumes a mirrored
    selfie, so your right hand is "Left")."""

    name: str
    shoulder: int
    elbow: int
    wrist: int
    pinky: int
    index: int
    hand_label: str

    def label(self, mirror: bool) -> str:
        if not mirror:
            return self.hand_label
        return "Right" if self.hand_label == "Left" else "Left"


RIGHT = Side("right", RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST, RIGHT_PINKY, RIGHT_INDEX, "Left")
LEFT = Side("left", 11, 13, 15, 17, 19, "Right")

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

# The preview sent to the app (--stream-preview): this wide, at most this
# often, this JPEG quality - a few kB a frame.
PREVIEW_WIDTH = 360
PREVIEW_INTERVAL = 1 / 12
PREVIEW_QUALITY = 60


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
        self.left_shoulder = None  # the robot's left shoulder, if its left arm works
        self.kind = "?"
        self.assist = True  # the app's help reaching for things, told again on every connect
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
                left = hello.get("left_shoulder")
                self.left_shoulder = tuple(left) if left else None
            self.send({"type": "assist", "on": self.assist})
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
    # Finding a hand needs a palm in view; following one it already has
    # doesn't. So: a normal bar to find it, and a low one to keep it - a hand
    # turning edge-on, or moving fast, stays tracked instead of being dropped
    # and searched for again. (Picking *your* hand is right_hand's job.)
    hands = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path("hand_landmarker.task")),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.3,
            min_tracking_confidence=0.3,
        )
    )
    # For a second look, zoomed in on the hand, when the whole frame misses
    # it: one still image at a time (crops jump around, so no tracking).
    zoomed = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path("hand_landmarker.task")),
            running_mode=vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.4,
            min_hand_presence_confidence=0.3,
        )
    )
    return mp, pose, hands, zoomed


# A hand whose wrist is farther than this from the arm's wrist (fraction of
# the image) is someone else's, or the other hand.
HAND_NEAR_WRIST = 0.2


def right_hand(hand_result, pose_wrist, mirror: bool = False):
    """Which detected hand is your right one (see hand_at)."""
    return hand_at(hand_result, pose_wrist, RIGHT.label(mirror))


def hand_at(hand_result, pose_wrist, expected: str):
    """Which detected hand is the one at the pose's wrist in the image -
    preferring, among those, the one MediaPipe labels `expected`. Its labels
    assume a mirrored selfie image, so on a raw webcam frame your right hand
    is labelled "Left" (as Maestro's hand selection has it); the label only
    breaks ties, position decides."""
    if not hand_result.hand_landmarks or pose_wrist is None:
        return None, None
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


def find_hand(mp, cv2, detected, zoomed, rgb, seen, mirror, side: Side = RIGHT):
    """One of your hands this frame: (image landmarks, world landmarks,
    where from) - "hand" from the whole frame (`detected`, this frame's
    hand tracker result), "zoomed" from a second look at a crop around where
    the body tracker puts the hand, or (None, None, None).
    """
    expected = side.label(mirror)
    hand_image, hand_world = hand_at(detected, seen[side.wrist], expected)
    if hand_world is not None:
        return hand_image, hand_world, "hand"
    height, width = rgb.shape[:2]
    crop = hand_crop(seen[side.wrist], seen[side.index], seen[side.pinky], width, height)
    if crop is None:
        return None, None, None
    left, top, size = crop
    patch = cv2.resize(rgb[top : top + size, left : left + size], (256, 256))
    result = zoomed.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=patch))
    if not result.hand_landmarks:
        return None, None, None
    # Back to whole-frame coordinates (world landmarks don't change: they're
    # in metres around the hand, wherever it is in the picture).
    points = [
        SimpleNamespace(x=(left + p.x * size) / width, y=(top + p.y * size) / height) for p in result.hand_landmarks[0]
    ]
    mapped = SimpleNamespace(hand_landmarks=[points], hand_world_landmarks=result.hand_world_landmarks,
                             handedness=result.handedness)
    hand_image, hand_world = hand_at(mapped, seen[side.wrist], expected)
    return (hand_image, hand_world, "zoomed") if hand_world is not None else (None, None, None)


# ---- the main loop --------------------------------------------------------


class Controller:
    """Smoothing, arm-length averaging and the pause / lost-tracking rules.

    Each frame: ``hand()`` first (if the hand tracker sees it) - the grip,
    and which way the hand points and is turned - or else ``body_hand()``
    (the body tracker's rougher idea of the hand: which way, but no grip);
    then ``arm()``, which sends the reach, with the hand's tilt and twist
    when a hand was seen lately.
    """

    def __init__(self, robot: Robot, mirror: bool, follow_wrist: bool = True, side: Side = RIGHT) -> None:
        self.robot = robot
        self.side = side
        # Your left arm drives the robot's left arm, which is the right one
        # mirrored (z -> -z): everything is worked out as for the right arm
        # with left and right swapped, and the target mirrored back.
        self.left = side is LEFT
        self.mirror = mirror != self.left
        self.follow_wrist = follow_wrist
        # Positions in metres move at up to ~1.5 m/s: beta lets the filter
        # open right up for a quick move, so the robot doesn't trail it.
        # Forward / back (the robot's x) is the webcam's depth guess, by far
        # the noisiest: smoothed harder.
        self.filters = [OneEuroFilter(min_cutoff=0.6, beta=0.8)] + [
            OneEuroFilter(min_cutoff=1.0, beta=1.5) for _ in range(2)
        ]
        self.pitch_filter = OneEuroFilter(min_cutoff=1.0, beta=0.5)
        self.roll_filter = OneEuroFilter(min_cutoff=1.0, beta=0.5)
        self.grip_filter = OneEuroFilter(min_cutoff=1.5, beta=0.5)
        self.grip_gate = GripGate()
        self.grasp_hold = GraspHold()
        self.grip_amount = 0.0  # the filtered grip, before the gate
        self.holding_still = False  # the hand held while the grip changes
        self.arm_length: float | None = None
        self.heading = (1.0, 0.0)  # (forward, outward): facing straight ahead
        self.paused = False
        self.tracking = False
        self.grip = 0.0
        self.pitch: float | None = None  # the robot hand's tilt, None = left to the planner
        self.roll: float | None = None  # and its twist, None = left as it is
        self.hand_source: str | None = None  # where the hand's direction came from, for the preview
        self._hand_direction = None
        self._hand_across = None
        self._hand_seen = -math.inf

    def arm(self, shoulder, elbow, wrist, now: float) -> None:
        if self.left and self.robot.left_shoulder is None:
            return  # the robot has no working left arm
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
        fresh = self.follow_wrist and self._hand_direction is not None and now - self._hand_seen <= HAND_TIMEOUT
        if fresh:
            self.pitch = self._smooth(self.pitch_filter, self.pitch, hand_pitch(self._hand_direction, self.heading), now)
        else:
            self.pitch = None
            self.pitch_filter.reset()
        pitch = self.pitch
        if pitch is None:
            pitch = math.atan2(wrist_target[1] - self.robot.shoulder[1], math.hypot(forward, outward))
        if fresh:
            self.roll = self._smooth(self.roll_filter, self.roll, hand_roll(self._hand_across, self.heading, pitch), now)
        else:
            self.roll = None  # the robot's hand stays turned as it was
            self.roll_filter.reset()
        tip = hand_tip(wrist_target, self.heading, pitch, hand)

        self.tracking = True
        # Closing or opening the hand: the robot's stays where it is.
        self.holding_still = self.grasp_hold(now, self.grip_amount, self.grip_gate.closed)
        if not self.paused and not self.holding_still:
            message = {"type": "reach", "x": tip[0], "y": tip[1], "z": -tip[2] if self.left else tip[2]}
            if self.left:
                message["hand"] = "left"
            if self.pitch is not None:
                message["pitch"] = math.remainder(self.pitch, 2 * math.pi)
            if self.roll is not None:
                message["roll"] = math.remainder(self.roll, 2 * math.pi)
            self.robot.send(message)

    @staticmethod
    def _smooth(f: OneEuroFilter, last: float | None, angle: float, now: float) -> float:
        """An angle through its filter, unwrapped first, so the filter never
        sees a 2*pi jump (from +179 to -179 degrees, say)."""
        if last is not None:
            angle += 2 * math.pi * round((last - angle) / (2 * math.pi))
        return f(angle, now)

    def hand(self, landmarks, now: float, source: str = "hand") -> None:
        self._hand_direction = hand_direction(landmarks, self.mirror)
        self._hand_across = hand_across(landmarks, self.mirror)
        self._hand_seen = now
        self.hand_source = source
        self.grip_amount = self.grip_filter(grip_amount(landmarks), now)
        self.grip = self.grip_gate(self.grip_amount)
        if not self.paused and not (self.left and self.robot.left_shoulder is None):
            message = {"type": "grip", "amount": self.grip}
            if self.left:
                message["hand"] = "left"
            self.robot.send(message)

    def body_hand(self, wrist, index, pinky, now: float) -> None:
        """The hand tracker lost the hand, but the body tracker still has a
        rough idea of it: keep steering the tilt and twist from that. The
        grip stays as it was - three points can't tell open from closed."""
        self._hand_direction, self._hand_across = body_hand_frame(wrist, index, pinky, self.mirror)
        self._hand_seen = now
        self.hand_source = "body"

    def lost(self, hold: bool = True) -> None:
        """Out of view: stop where it is (once; `hold` False leaves the
        robot moving - one arm out of view while the other still steers),
        and start smoothing afresh."""
        if self.tracking:
            if hold:
                self.robot.send({"type": "hold"})
            for f in self.filters:
                f.reset()
            self.pitch_filter.reset()
            self.roll_filter.reset()
            self.pitch = None
            self.roll = None
        self.tracking = False

    def toggle_assist(self) -> None:
        self.robot.assist = not self.robot.assist
        self.robot.send({"type": "assist", "on": self.robot.assist})

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        if self.paused:
            self.robot.send({"type": "hold"})


def run_camera(args, robot: Robot) -> None:
    import cv2

    mp, pose, hands, zoomed = make_trackers(args.pose_model)
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        sys.exit(f"can't open camera {args.camera} (on macOS, allow the terminal to use the camera)")
    control = Controller(robot, args.mirror, follow_wrist=not args.no_wrist)
    left_control = Controller(robot, args.mirror, follow_wrist=not args.no_wrist, side=LEFT)
    controls = ((RIGHT, control), (LEFT, left_control))
    start = time.monotonic()
    stamp = -1
    last_preview = -math.inf

    while True:
        ok, frame = camera.read()
        if not ok:
            break
        robot.ensure_connected()
        now = time.monotonic() - start
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        # VIDEO mode refuses a timestamp that doesn't move forward.
        stamp = max(int(now * 1000), stamp + 1)
        body = pose.detect_for_video(image, stamp)
        found = bool(body.pose_world_landmarks)
        seen = body.pose_landmarks[0] if found else None
        tracked = []  # the arms steering this frame
        hand_images = []
        if found:
            world = body.pose_world_landmarks[0]
            # Only steer from a clearly visible arm.
            visible = [(side, c) for side, c in controls
                       if min(seen[i].visibility for i in (side.shoulder, side.elbow, side.wrist)) > 0.5]
            detected = hands.detect_for_video(image, stamp) if visible else None
            for side, c in visible:
                hand_image, hand_world, source = find_hand(mp, cv2, detected, zoomed, rgb, seen, args.mirror, side)
                if hand_world is not None:
                    c.hand(hand_world, now, source)
                    hand_images.append(hand_image)
                elif min(seen[i].visibility for i in (side.index, side.pinky)) > 0.5:
                    c.body_hand(world[side.wrist], world[side.index], world[side.pinky], now)
                c.arm(world[side.shoulder], world[side.elbow], world[side.wrist], now)
                tracked.append(side)
        # An arm out of view stops; the robot holds still only when neither is in view.
        for side, c in controls:
            if side not in tracked:
                c.lost(hold=not tracked)

        stream = args.stream_preview and now - last_preview >= PREVIEW_INTERVAL
        if stream or not args.no_preview:
            view = draw_preview(cv2, frame, seen, hand_images, control, tracked, left_control)
        if stream:
            last_preview = now
            robot.send(preview_message(cv2, view))
        if not args.no_preview:
            cv2.imshow("Robot arm - webcam control (space: pause, a: assist, q: quit)", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                control.toggle_pause()
                left_control.paused = control.paused
            if key == ord("a"):
                control.toggle_assist()

    camera.release()
    robot.close()


def preview_message(cv2, view) -> dict:
    """A preview image as a message for the app: small, JPEG, base64."""
    height, width = view.shape[:2]
    small = cv2.resize(view, (PREVIEW_WIDTH, max(1, round(height * PREVIEW_WIDTH / width))))
    _, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, PREVIEW_QUALITY])
    return {"type": "preview", "jpeg": base64.b64encode(jpeg.tobytes()).decode("ascii")}


def draw_preview(cv2, frame, pose_landmarks, hands, control: Controller, arms=(RIGHT,), left: Controller | None = None):
    """The camera image as a mirror (what people expect), with the tracked
    arms (`arms`) and hands (`hands`, landmark lists), the grip, and the
    state of things."""
    height, width = frame.shape[:2]
    view = cv2.flip(frame, 1)

    def pixel(p):
        return int((1.0 - p.x) * width), int(p.y * height)  # flipped like the view

    if pose_landmarks is not None:
        for side in arms:
            points = [pixel(pose_landmarks[i]) for i in (side.shoulder, side.elbow, side.wrist)]
            color = (255, 224, 23) if side is RIGHT else (60, 176, 255)
            cv2.line(view, points[0], points[1], color, 4)
            cv2.line(view, points[1], points[2], color, 4)
            for p in points:
                cv2.circle(view, p, 7, (255, 255, 255), -1)
    for hand in hands:
        for p in hand:
            cv2.circle(view, pixel(p), 3, (60, 200, 255), -1)

    # Grip bar: open (left) .. closed (right).
    cv2.rectangle(view, (20, height - 40), (220, height - 20), (80, 80, 80), 2)
    cv2.rectangle(view, (20, height - 40), (20 + int(200 * control.grip), height - 20), (60, 200, 255), -1)
    cv2.line(view, (20 + int(200 * 0.7), height - 44), (20 + int(200 * 0.7), height - 16), (255, 255, 255), 1)
    grip_text = "grip: CLOSED" if control.grip_gate.closed else "grip: open"
    if control.holding_still:
        grip_text += " - holding still"
    if left is not None and LEFT in arms and control.robot.left_shoulder is not None:
        grip_text += " | left: " + ("CLOSED" if left.grip_gate.closed else "open")
    cv2.putText(view, grip_text, (230, height - 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if control.pitch is not None:
        sources = {"hand": "", "zoomed": " (zoomed in)", "body": " (body tracker: no grip)"}
        tilt = f"hand tilt {math.degrees(control.pitch):+.0f}, twist {math.degrees(control.roll or 0.0):+.0f} deg"
        tilt += sources.get(control.hand_source, "")
    else:
        tilt = "hand tilt / twist: free (show your hand)" if control.follow_wrist else "hand tilt / twist: off"
    cv2.putText(view, tilt, (20, height - 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if not control.robot.connected:
        status, color = "waiting for the robot app...", (0, 165, 255)
    elif control.paused:
        status, color = "PAUSED - space to resume", (0, 200, 255)
    elif not control.tracking and not (left is not None and left.tracking):
        status, color = "step into view: shoulders and arms", (0, 165, 255)
    else:
        status, color = "controlling the robot", (80, 220, 80)
    cv2.putText(view, status, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    assist = "assist: on (a: off)" if control.robot.assist else "assist: off (a: on)"
    cv2.putText(view, assist, (20, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return view


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
    parser.add_argument("--stream-preview", action="store_true",
                        help="send the preview to the app instead (the app starts this program that way)")
    parser.add_argument("--no-wrist", action="store_true",
                        help="don't copy your hand's tilt and twist: leave the robot's wrist to its planner")
    parser.add_argument("--no-assist", action="store_true",
                        help="no help from the app: it won't guess what you reach for or finish your grabs")
    parser.add_argument("--pose-model", choices=("lite", "full", "heavy"), default="full",
                        help="body tracker: lite is fastest, heavy judges depth best (default full)")
    parser.add_argument("--demo", type=float, nargs="?", const=30.0, metavar="SECONDS",
                        help="no camera: drive the robot with a scripted arm for SECONDS (default 30)")
    args = parser.parse_args()

    robot = Robot(f"ws://{args.host}:{args.port}")
    robot.assist = not args.no_assist
    try:
        if args.demo is not None:
            run_demo(robot, args.demo)
        else:
            run_camera(args, robot)
    except KeyboardInterrupt:
        robot.close()


if __name__ == "__main__":
    main()
