"""The math of webcam control, kept apart from the camera so it can be tested.

These steps turn what MediaPipe sees into robot commands:

1. ``arm_target``     - your wrist relative to your shoulder, scaled from your
                        arm's length to the robot's, becomes the point the
                        robot's wrist goes to.
2. ``steady_heading`` - keeps the robot's waist from spinning when your hand
                        is close to your body, where the direction to it is
                        mostly tracking noise.
3. ``hand_pitch``     - how far your hand is tipped up or down; ``hand_tip``
                        then puts the robot's hand on its wrist, tipped the
                        same way. ``hand_roll`` - how far your palm is turned
                        (the forearm's twist). Both work from the hand
                        tracker's landmarks or, when it loses your hand, the
                        body tracker's rougher hand points
                        (``body_hand_frame``).
4. ``grip_amount``    - how bent your fingers are, and how far their tips are
                        from your wrist, becomes 0 (open) .. 1 (fist);
                        ``GripGate`` only lets it grab or let go once a few
                        frames in a row agree.
5. ``OneEuroFilter``  - removes the frame-to-frame jitter of the tracking
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


def to_robot(dx: float, dy: float, dz: float, mirror: bool = False) -> tuple[float, float, float]:
    """A direction seen by the camera as the robot's (forward, up, outward).

    MediaPipe's world landmarks have x toward the image's right, y down and z
    away from the camera. The robot faces +X with +Y up and its right arm on
    the +Z side, so, for a person facing the camera:

    * toward the camera (z decreasing)         -> forward (+X)
    * up (y decreasing)                        -> up (+Y)
    * out to your right, which is the image's
      left in an unmirrored frame              -> out to the robot's right (+Z)
    """
    outward = -dx
    return -dz, -dy, -outward if mirror else outward


def arm_target(
    shoulder: Point,
    elbow: Point,
    wrist: Point,
    robot_shoulder: Sequence[float],
    robot_reach: float,
    arm_length: float | None = None,
    mirror: bool = False,
    robot_hand: float = 0.0,
) -> tuple[float, float, float]:
    """Where the robot's wrist should go, in the app's world (metres).

    The inputs are MediaPipe *world* landmarks of your right arm (metres,
    centred between your hips; see ``to_robot`` for the directions).

    Your arm is scaled to the robot's - shoulder to wrist, without the hand
    (``robot_reach`` less ``robot_hand``) - so a fully stretched arm asks for
    the robot's full reach whatever your size. ``arm_length`` lets the caller
    pass a steadier (averaged) length than one frame's; ``mirror`` flips
    left/right if the camera image is mirrored.
    """
    length = arm_length if arm_length else distance(shoulder, elbow) + distance(elbow, wrist)
    scale = (robot_reach - robot_hand) / max(length, 1e-6)
    forward, up, outward = to_robot(wrist.x - shoulder.x, wrist.y - shoulder.y, wrist.z - shoulder.z, mirror)
    return (
        robot_shoulder[0] + forward * scale,
        robot_shoulder[1] + up * scale,
        robot_shoulder[2] + outward * scale,
    )


def steady_heading(
    forward: float, outward: float, heading: tuple[float, float], min_radius: float = 0.12
) -> tuple[float, float, tuple[float, float]]:
    """Keep the robot's waist still when the hand is close to the body.

    The robot's arm only bends in one plane, so its waist turns to face the
    hand: the waist's angle is the direction of the hand's horizontal offset
    (``forward``, ``outward``) from the shoulder. Near the body that offset is
    a few centimetres and mostly tracking noise - its direction swings wildly,
    and so would the waist. Inside ``min_radius`` (m) the offset keeps the
    last good direction, ``heading`` (a unit (forward, outward)), at
    ``min_radius``: the waist holds still and the hand stays just in front of
    it. Returns the offset to use and the heading to keep for next time.
    """
    radius = math.hypot(forward, outward)
    if radius >= min_radius:
        heading = (forward / radius, outward / radius)
        return forward, outward, heading
    return heading[0] * min_radius, heading[1] * min_radius, heading


# MediaPipe hand landmark indices.
WRIST = 0
MIDDLE_KNUCKLE = 9
FINGERTIPS = (8, 12, 16, 20)  # index, middle, ring, pinky
INDEX_KNUCKLE, PINKY_KNUCKLE = 5, 17
# Each finger's knuckle, middle joint, last joint and tip.
FINGERS = ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))


def hand_direction(hand: Sequence[Point], mirror: bool = False) -> tuple[float, float, float]:
    """Which way the hand points - wrist to the middle knuckle, so the same
    open or closed - as the robot's (forward, up, outward)."""
    w, k = hand[WRIST], hand[MIDDLE_KNUCKLE]
    return to_robot(k.x - w.x, k.y - w.y, k.z - w.z, mirror)


def hand_across(hand: Sequence[Point], mirror: bool = False) -> tuple[float, float, float]:
    """Across the palm toward the thumb - pinky knuckle to index knuckle - as
    the robot's (forward, up, outward). The same for a left or right hand,
    open or closed."""
    i, k = hand[INDEX_KNUCKLE], hand[PINKY_KNUCKLE]
    return to_robot(i.x - k.x, i.y - k.y, i.z - k.z, mirror)


def body_hand_frame(
    wrist: Point, index: Point, pinky: Point, mirror: bool = False
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """The hand's direction and across-the-palm vector (as ``hand_direction``
    and ``hand_across``) from the body tracker's three hand points - rougher
    than the hand tracker's 21, but there when it loses the hand."""
    mid = ((index.x + pinky.x) / 2, (index.y + pinky.y) / 2, (index.z + pinky.z) / 2)
    direction = to_robot(mid[0] - wrist.x, mid[1] - wrist.y, mid[2] - wrist.z, mirror)
    across = to_robot(index.x - pinky.x, index.y - pinky.y, index.z - pinky.z, mirror)
    return direction, across


def hand_pitch(direction: Sequence[float], heading: tuple[float, float]) -> float:
    """How far the hand is tipped up from level (radians, + up), in the
    vertical plane the robot's arm moves in (the one facing ``heading``).

    0 is level and pointing away from the body, +pi/2 straight up, -pi/2
    straight down; past that it's bent back toward the body.
    """
    forward, up, outward = direction
    along = forward * heading[0] + outward * heading[1]
    return math.atan2(up, along)


def hand_roll(across: Sequence[float], heading: tuple[float, float], pitch: float) -> float:
    """How far the palm is turned about the hand's own axis (radians), the
    way the robot's hand turns: 0 = thumb up, palm toward the body (holding
    a bottle); + turns the palm up (thumb out, away from the body), - down.

    ``across`` points to the thumb side; it's measured against the robot
    hand's own "up" (``pitch`` tipped, in the plane facing ``heading``) and
    its "out of the plane, away from the body".
    """
    forward, up, outward = across
    s, c = math.sin(pitch), math.cos(pitch)
    hand_up = (-s * heading[0], c, -s * heading[1])  # forward, up, outward
    away = (-heading[1], 0.0, heading[0])
    along_up = forward * hand_up[0] + up * hand_up[1] + outward * hand_up[2]
    along_away = forward * away[0] + outward * away[2]
    return math.atan2(along_away, along_up)


def hand_tip(
    wrist: Sequence[float], heading: tuple[float, float], pitch: float, robot_hand: float
) -> tuple[float, float, float]:
    """The robot's end-effector point: ``robot_hand`` (m) out from its wrist,
    tipped ``pitch`` up from level, in the plane facing ``heading``."""
    level = robot_hand * math.cos(pitch)
    return (
        wrist[0] + level * heading[0],
        wrist[1] + robot_hand * math.sin(pitch),
        wrist[2] + level * heading[1],
    )


def _clamp01(value: float) -> float:
    return min(1.0, max(0.0, value))


def _bend(a: Point, b: Point, c: Point) -> float:
    """How far the bone b->c turns away from a->b (radians, 0 = straight)."""
    u = (b.x - a.x, b.y - a.y, b.z - a.z)
    v = (c.x - b.x, c.y - b.y, c.z - b.z)
    nu, nv = math.hypot(*u), math.hypot(*v)
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    cos = sum(p * q for p, q in zip(u, v)) / (nu * nv)
    return math.acos(min(1.0, max(-1.0, cos)))


def finger_curl(hand: Sequence[Point], open_bend: float = 0.6, closed_bend: float = 2.6) -> float:
    """0 = straight fingers .. 1 = curled into a fist, from the joint angles.

    Each finger's bend at its knuckle plus at its middle joint (Maestro's
    joint-angle features): about 0.3 rad straight, about 3 rad in a fist.
    Angles don't change with distance from the camera or with which way the
    hand faces, where fingertip distances alone can be fooled by a hand seen
    edge-on.
    """
    total = 0.0
    for knuckle, middle, last, _tip in FINGERS:
        total += _bend(hand[WRIST], hand[knuckle], hand[middle]) + _bend(hand[knuckle], hand[middle], hand[last])
    return _clamp01((total / len(FINGERS) - open_bend) / (closed_bend - open_bend))


def fingertip_spread(hand: Sequence[Point], open_ratio: float = 1.9, closed_ratio: float = 1.1) -> float:
    """0 = open hand .. 1 = fist, from how far the fingertips are from the wrist.

    Fingertip-to-wrist distance divided by the palm's length (wrist to the
    middle knuckle): about 2 for an open hand, about 1 for a fist - the same at
    any distance from the camera, because it's a ratio.
    """
    palm = max(distance(hand[WRIST], hand[MIDDLE_KNUCKLE]), 1e-6)
    spread = sum(distance(hand[tip], hand[WRIST]) for tip in FINGERTIPS) / (len(FINGERTIPS) * palm)
    return _clamp01((open_ratio - spread) / (open_ratio - closed_ratio))


def grip_amount(hand: Sequence[Point]) -> float:
    """0 = open hand .. 1 = fist: the finger joints' bend and the fingertips'
    distance from the wrist, averaged - each covers for the other's bad
    angles (a hand seen edge-on, a joint the tracker guessed wrong)."""
    return 0.5 * (finger_curl(hand) + fingertip_spread(hand))


def hand_crop(
    wrist: Point, index: Point, pinky: Point, width: int, height: int, scale: float = 4.0, smallest: int = 128
) -> tuple[int, int, int] | None:
    """A square around the hand, in pixels (left, top, side), from the body
    tracker's hand points (normalised image coordinates) - to look for the
    hand again, zoomed in, when the whole frame misses it.

    The hand tracker first has to find a palm. Small in the frame, or turned
    edge-on, it often can't; filling a crop it usually can (MediaPipe
    Holistic finds hands the same way, from the body's pose). None if the
    hand is out of the picture.
    """
    cx = (wrist.x + index.x + pinky.x) / 3 * width
    cy = (wrist.y + index.y + pinky.y) / 3 * height
    reach = max(math.dist((wrist.x * width, wrist.y * height), (p.x * width, p.y * height)) for p in (index, pinky))
    side = int(max(smallest, scale * reach))
    side = min(side, width, height)
    left = int(min(max(cx - side / 2, 0), width - side))
    top = int(min(max(cy - side / 2, 0), height - side))
    if not (0 <= cx < width and 0 <= cy < height):
        return None
    return left, top, side


class GripGate:
    """Grab and let go only when the hand clearly means it.

    The app grabs when the grip reaches ``grab_at`` and lets go when it drops
    to ``release_at``. A tracker glitch can cross either for a frame, so - like
    Maestro's temporal voting, which accepts a gesture only once several
    frames in a row agree - the grip may only cross after ``frames``
    consecutive frames past it; until then it is held just short. Between the
    two, it passes straight through, so the fingers still follow your hand.
    """

    def __init__(self, grab_at: float = 0.7, release_at: float = 0.3, frames: int = 3) -> None:
        self.grab_at = grab_at
        self.release_at = release_at
        self.frames = frames
        self.closed = False
        self._agreeing = 0

    def __call__(self, amount: float) -> float:
        crossing = amount <= self.release_at if self.closed else amount >= self.grab_at
        self._agreeing = self._agreeing + 1 if crossing else 0
        if crossing and self._agreeing >= self.frames:
            self.closed = not self.closed
            self._agreeing = 0
            return amount
        if self.closed:
            return max(amount, self.release_at + 0.01)
        return min(amount, self.grab_at - 0.01)


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
