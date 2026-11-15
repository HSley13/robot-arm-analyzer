"""The math of webcam control, kept apart from the camera so it can be tested.

Three steps turn what MediaPipe sees into robot commands:

1. ``arm_target``    - your wrist relative to your shoulder, scaled from your
                       arm's length to the robot's, becomes the point the
                       robot's hand reaches for.
2. ``grip_amount``   - how far your fingertips are from your wrist, relative to
                       your palm's size, becomes 0 (open) .. 1 (fist).
3. ``OneEuroFilter`` - removes the frame-to-frame jitter of the tracking
                       without making the robot lag when you move quickly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence


class Point(Protocol):
    """Anything with x, y, z - MediaPipe landmarks are."""

    x: float
    y: float
    z: float


@dataclass
class P:
    """A plain point, for tests and for the demo mode."""

    x: float
    y: float
    z: float


def distance(a: Point, b: Point) -> float:
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def arm_target(
    shoulder: Point,
    elbow: Point,
    wrist: Point,
    robot_shoulder: Sequence[float],
    robot_reach: float,
    arm_length: float | None = None,
    mirror: bool = False,
) -> tuple[float, float, float]:
    """Where the robot's hand should go, in the app's world (metres).

    The inputs are MediaPipe *world* landmarks of your right arm: metres,
    centred between your hips, x toward the image's right, y down, z away from
    the camera. The robot faces +X with +Y up and its right arm on the +Z side,
    so, for a person facing the camera:

    * reaching toward the camera (z decreasing)  -> robot reaches forward (+X)
    * raising the hand (y decreasing)            -> robot raises its hand (+Y)
    * moving the hand out to your right, which is
      the image's left in an unmirrored frame    -> robot moves out to its right (+Z)

    Your arm is scaled to the robot's (``robot_reach`` / your arm length), so a
    fully stretched arm asks for the robot's full reach whatever your size.
    ``arm_length`` lets the caller pass a steadier (averaged) length than one
    frame's; ``mirror`` flips left/right if the camera image is mirrored.
    """
    length = arm_length if arm_length else distance(shoulder, elbow) + distance(elbow, wrist)
    scale = robot_reach / max(length, 1e-6)
    forward = -(wrist.z - shoulder.z) * scale
    up = -(wrist.y - shoulder.y) * scale
    outward = -(wrist.x - shoulder.x) * scale
    if mirror:
        outward = -outward
    return (
        robot_shoulder[0] + forward,
        robot_shoulder[1] + up,
        robot_shoulder[2] + outward,
    )


# MediaPipe hand landmark indices.
WRIST = 0
MIDDLE_KNUCKLE = 9
FINGERTIPS = (8, 12, 16, 20)  # index, middle, ring, pinky


def grip_amount(hand: Sequence[Point], open_ratio: float = 1.9, closed_ratio: float = 1.1) -> float:
    """0 = open hand .. 1 = fist.

    Fingertip-to-wrist distance divided by the palm's length (wrist to the
    middle knuckle): about 2 for an open hand, about 1 for a fist - the same at
    any distance from the camera, because it's a ratio.
    """
    palm = max(distance(hand[WRIST], hand[MIDDLE_KNUCKLE]), 1e-6)
    spread = sum(distance(hand[tip], hand[WRIST]) for tip in FINGERTIPS) / (len(FINGERTIPS) * palm)
    amount = (open_ratio - spread) / (open_ratio - closed_ratio)
    return min(1.0, max(0.0, amount))


class OneEuroFilter:
    """The 1€ filter (Casiez, Roussel and Vogel, 2012) for one number.

    A low-pass filter whose cutoff rises with speed: holding still, it smooths
    hard (no jitter); moving fast, it barely smooths (no lag). ``min_cutoff``
    sets the smoothing at rest (lower = smoother), ``beta`` how quickly that
    relaxes with speed (higher = less lag).
    """

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.3, d_cutoff: float = 1.0) -> None:
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._value: float | None = None
        self._speed = 0.0
        self._time: float | None = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, value: float, time: float) -> float:
        if self._value is None or self._time is None:
            self._value, self._time = value, time
            return value
        dt = max(time - self._time, 1e-3)
        self._time = time
        # How fast the value changes (itself smoothed), then smooth the value
        # with a cutoff that rises with that speed.
        a_d = self._alpha(self.d_cutoff, dt)
        self._speed = a_d * (value - self._value) / dt + (1.0 - a_d) * self._speed
        a = self._alpha(self.min_cutoff + self.beta * abs(self._speed), dt)
        self._value = a * value + (1.0 - a) * self._value
        return self._value

    def reset(self) -> None:
        self._value = None
        self._time = None
        self._speed = 0.0
