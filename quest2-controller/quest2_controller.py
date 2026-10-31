#!/usr/bin/env python3
"""Meta Quest 2 controller for the Waveshare RoArm-M3 over USB serial.

How it works:
- This program runs a small HTTPS + WebSocket server.
- On the Quest 2, open the page it serves in the Quest Browser and press
  "Start (passthrough)". The page is a WebXR app: it streams the pose and
  buttons of the controllers back to this program about 70 times a second.
- While you hold the grip button, the gripper tip of the arm follows the
  controller: moving the controller moves the tip (X/Y/Z), tilting and
  rolling it tilts and rolls the tip, and the trigger closes the gripper.
  The arm is moved with {"T":1041} commands (inverse kinematics, no blocking).
- Releasing the grip button freezes the arm where it is. You can then move
  your hand somewhere comfortable and squeeze again to carry on ("clutch").
- Movements can be recorded (X on the other controller) and played back
  (Y) once, a number of times, or forever. Recordings are saved in
  recordings/ and can also be chosen and played from the web page.

Usage:
    python quest2_controller.py --port /dev/ttyUSB0
    python quest2_controller.py --dry-run            # no arm, print commands
"""

import argparse
import asyncio
import datetime
import ipaddress
import json
import math
import signal
import socket
import ssl
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path

from aiohttp import WSMsgType, web

try:
    import serial
except ImportError:  # only needed when talking to the arm
    serial = None


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
DEFAULT_SCALE = 1.0              # mm the arm moves per mm the controller moves
LOOP_HZ = 50                     # control loop rate
SEND_HZ = 25                     # max T:1041 commands per second

# The arm never moves faster than this, however fast the controller moves.
# T:1041 moves to each target "at the fastest speed", so the controller
# feeds it small steps instead.
MAX_LINEAR_SPEED = 250.0                 # mm/s for X/Y/Z
MAX_ANGULAR_SPEED = math.radians(120)    # rad/s for pitch, roll and gripper

# Time constant of the low-pass filter on the controller pose. Larger values
# hide more hand tremor but make the arm lag further behind.
SMOOTHING = 0.06                 # s

# Stop following if no controller update arrives for this long
# (headset taken off, tracking lost, WiFi hiccup).
POSE_TIMEOUT = 0.3               # s

# Grip button (squeeze) thresholds, 0..1. Engages above GRIP_ON and releases
# below GRIP_OFF, so a half-held grip doesn't flicker on and off.
GRIP_ON = 0.6
GRIP_OFF = 0.3

# Workspace of the gripper tip. The docs give no limits, so these are cautious
# guesses: widen them once you know your arm. The tip stays inside a ring
# around the base (MIN_RADIUS..MAX_RADIUS from the base axis, in mm), between
# Z_MIN and Z_MAX, and within BASE_ANGLE_LIMIT either side of straight ahead.
MIN_RADIUS = 100.0
MAX_RADIUS = 450.0
Z_MIN = -50.0
Z_MAX = 450.0
BASE_ANGLE_LIMIT = math.radians(120)

# Pitch, roll and gripper ranges (rad), from the joint ranges in the docs.
PITCH_RANGE = (-1.57, 1.57)
ROLL_RANGE = (-3.14, 3.14)
GRIPPER_RANGE = (1.08, 3.14)

# Gripper angles for a released and a fully pulled trigger. 3.14 is closed,
# smaller is more open (1.08 is the widest).
GRIPPER_OPEN = 1.57
GRIPPER_CLOSED = 3.14
# Analog gripper: after you squeeze the grip button, the gripper doesn't follow
# the trigger until the trigger reaches or passes the gripper's current angle
# (give or take this much, in rad). This stops a held object being dropped
# when you start moving.
GRIPPER_TAKEOVER = 0.15

# Direction of pitch and roll. Pitch: tilting the controller nose-down tilts the
# tip down. Roll: rolling the controller clockwise rolls the tip clockwise.
# Set to -1 to flip one, or 0 to ignore it.
PITCH_SIGN = -1
ROLL_SIGN = +1

# Position after T:100 (from the feedback example in the docs). Only used
# when the arm can't report its position (e.g. --dry-run).
HOME_POSE_VALUES = dict(x=345.0, y=0.0, z=207.0, t=0.0, r=0.0, g=3.14)
HOME_WAIT = 3.0                  # s to wait after T:100 before following again

# When engaging, the arm's reported position is used unless it is this close
# to the last commanded target, in which case the target is kept. That stops a
# loaded arm, which sags a little, from creeping down each time you engage.
RESYNC_TOLERANCE_MM = 20.0
RESYNC_TOLERANCE_RAD = 0.2

# Haptic feedback on the controller: (intensity 0..1, duration ms).
HAPTIC_ENGAGE = (0.6, 40)
HAPTIC_RELEASE = (0.3, 25)
HAPTIC_LIMIT = (0.25, 30)        # repeated while you push past a limit
HAPTIC_LIMIT_EVERY = 0.25        # s
HAPTIC_ERROR = (1.0, 150)
HAPTIC_TICK = (0.4, 15)          # gripper takes over, orientation toggled

HAPTIC_RECORD = (0.8, 120)       # recording started or stopped (other controller)
HAPTIC_PLAY = (0.5, 60)          # playback started or finished

# Playback: before each loop the arm glides to where the recording started,
# at up to these speeds, then waits PLAYBACK_SETTLE before replaying.
PLAYBACK_LINEAR_SPEED = 100.0            # mm/s
PLAYBACK_ANGULAR_SPEED = math.radians(45)  # rad/s
PLAYBACK_SETTLE = 0.5                    # s

RECORDING_FORMAT = "roarm-m3-quest2-recording"
RECORDING_VERSION = 1

DEFAULT_LISTEN_PORT = 8443
APP_DIR = Path(__file__).resolve().parent
WEB_DIR = APP_DIR / "web"
CERT_DIR = APP_DIR / "certs"
RECORDINGS_DIR = APP_DIR / "recordings"


# ---------------------------------------------------------------------------
# Arm pose
# ---------------------------------------------------------------------------
@dataclass
class Pose:
    """Gripper tip pose, as used by T:1041 and reported by T:105."""
    x: float  # mm, forward
    y: float  # mm, left
    z: float  # mm, up
    t: float  # rad, pitch
    r: float  # rad, roll
    g: float  # rad, gripper

    def copy(self):
        return replace(self)

    def command(self):
        """T:1041 command that moves the tip straight to this pose."""
        return {"T": 1041, "x": round(self.x, 1), "y": round(self.y, 1), "z": round(self.z, 1),
                "t": round(self.t, 4), "r": round(self.r, 4), "g": round(self.g, 4)}

    def as_dict(self):
        return {"x": round(self.x, 1), "y": round(self.y, 1), "z": round(self.z, 1),
                "t": round(self.t, 3), "r": round(self.r, 3), "g": round(self.g, 3)}

    def differs(self, other, mm=0.05, rad=1e-4):
        return (max(abs(self.x - other.x), abs(self.y - other.y), abs(self.z - other.z)) > mm
                or max(abs(self.t - other.t), abs(self.r - other.r), abs(self.g - other.g)) > rad)

    def close_to(self, other):
        """Same position and tip angle, give or take the RESYNC tolerances.
        The gripper is ignored: it reads back more open when it holds an object."""
        return (math.dist((self.x, self.y, self.z), (other.x, other.y, other.z)) <= RESYNC_TOLERANCE_MM
                and max(abs(self.t - other.t), abs(self.r - other.r)) <= RESYNC_TOLERANCE_RAD)


HOME_POSE = Pose(**HOME_POSE_VALUES)


def pose_from_feedback(fb, fallback):
    """Pose from a T:1051 feedback message. Missing fields come from fallback."""
    def get(key, default):
        try:
            value = float(fb[key])
        except (KeyError, TypeError, ValueError):
            return default
        return value if math.isfinite(value) else default
    return Pose(get("x", fallback.x), get("y", fallback.y), get("z", fallback.z),
                get("tit", fallback.t), get("r", fallback.r), get("g", fallback.g))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def clamp_position(x, y, z):
    """Closest point inside the workspace (near enough for small steps)."""
    radius = clamp(math.hypot(x, y), MIN_RADIUS, MAX_RADIUS)
    angle = clamp(math.atan2(y, x), -BASE_ANGLE_LIMIT, BASE_ANGLE_LIMIT)
    return radius * math.cos(angle), radius * math.sin(angle), clamp(z, Z_MIN, Z_MAX)


def step_within_range(old, new, lo, hi):
    """Move from old to new without leaving lo..hi. A value that is already
    outside the range (e.g. read from the arm) can move back in but not further out."""
    if new > old:
        return min(new, max(hi, old))
    return max(new, min(lo, old))


ANGLE_RANGES = {"t": PITCH_RANGE, "r": ROLL_RANGE, "g": GRIPPER_RANGE}


def outside_workspace(pose, tolerance=5.0):
    """Description of how pose is outside the workspace, or None if it is inside."""
    x, y, z = clamp_position(pose.x, pose.y, pose.z)
    if math.dist((x, y, z), (pose.x, pose.y, pose.z)) > tolerance:
        return (f"the tip is at x={pose.x:.0f} y={pose.y:.0f} z={pose.z:.0f} mm, "
                f"outside the workspace limits")
    return None


def past_limit(desired):
    """True if desired lies outside the workspace or the angle ranges."""
    x, y, z = clamp_position(desired.x, desired.y, desired.z)
    if math.dist((x, y, z), (desired.x, desired.y, desired.z)) > 1.0:
        return True
    return any(not lo - 1e-3 <= getattr(desired, f) <= hi + 1e-3 for f, (lo, hi) in ANGLE_RANGES.items())


# ---------------------------------------------------------------------------
# Controller pose maths (WebXR space: +X right, +Y up, -Z forward, metres)
# ---------------------------------------------------------------------------
def rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    tx = 2 * (qy * v[2] - qz * v[1])
    ty = 2 * (qz * v[0] - qx * v[2])
    tz = 2 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def pitch_roll(q):
    """Pitch (nose up = +) and roll (clockwise seen from behind = +) of an
    orientation, measured against gravity, so they don't depend on which way
    you face."""
    forward = rotate(q, (0.0, 0.0, -1.0))
    up = rotate(q, (0.0, 1.0, 0.0))
    right = rotate(q, (1.0, 0.0, 0.0))
    return math.asin(clamp(forward[1], -1.0, 1.0)), math.atan2(-right[1], up[1])


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def horizontal_frame(q):
    """Unit (forward, right) vectors on the floor for where q is looking."""
    f = rotate(q, (0.0, 0.0, -1.0))
    n = math.hypot(f[0], f[2])
    if n < 1e-3:  # looking straight up or down: use the space's own forward
        return (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)
    fx, fz = f[0] / n, f[2] / n
    return (fx, 0.0, fz), (-fz, 0.0, fx)


@dataclass
class Hand:
    """One controller, from the latest message sent by the page."""
    pos: tuple        # grip position (m)
    quat: tuple       # pointing (target ray) orientation (x, y, z, w)
    trigger: float    # 0..1
    grip: float       # 0..1
    a: bool           # A or X button
    b: bool           # B or Y button


def _floats(values, n):
    out = tuple(float(v) for v in values)
    if len(out) != n or not all(math.isfinite(v) for v in out):
        raise ValueError("bad vector")
    return out


def parse_hand(data):
    """Hand from the page's JSON, or None if it is missing or malformed."""
    try:
        quat = _floats(data["quat"], 4)
        norm = math.sqrt(sum(c * c for c in quat))
        if norm < 1e-6:
            return None
        return Hand(pos=_floats(data["pos"], 3), quat=tuple(c / norm for c in quat),
                    trigger=clamp(float(data.get("trigger", 0.0)), 0.0, 1.0),
                    grip=clamp(float(data.get("grip", 0.0)), 0.0, 1.0),
                    a=bool(data.get("a")), b=bool(data.get("b")))
    except (KeyError, TypeError, ValueError):
        return None


@dataclass
class Anchor:
    """Controller and arm poses at the moment the grip was squeezed."""
    hand_pos: tuple
    forward: tuple    # horizontal direction you were looking (WebXR space)
    right: tuple
    pitch: float
    roll: float       # roll accumulated since engaging, unwrapped
    last_roll: float
    arm: Pose


# ---------------------------------------------------------------------------
# Links to the arm
# ---------------------------------------------------------------------------
def _encode(cmd):
    return json.dumps(cmd, separators=(",", ":"))


def _parse_feedback(text):
    """Return the first T:1051 feedback message found in text, or None."""
    for line in text.splitlines():
        try:
            msg = json.loads(line.strip())
        except ValueError:
            continue
        if isinstance(msg, dict) and msg.get("T") == 1051:
            return msg
    return None


class DryRunLink:
    """Prints commands instead of sending them."""

    simulated = True
    label = "dry run"

    def send(self, cmd):
        print("TX", _encode(cmd))

    def request_feedback(self, timeout=1.0):
        return None

    def close(self):
        pass


class SerialLink:
    """Sends JSON commands over USB serial and reads feedback in a background thread."""

    simulated = False

    def __init__(self, port, baud):
        if serial is None:
            sys.exit("pyserial is not installed. Run: pip install -r requirements.txt")
        self.label = port
        self.feedback = None
        self.feedback_event = threading.Event()
        self._lock = threading.Lock()
        self._request_lock = threading.Lock()  # one position request at a time
        self._running = True
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=None)
        except serial.SerialException as e:
            sys.exit(f"Can't open {port}: {e}")
        # Avoid resetting the ESP32 when the port opens.
        try:
            self._ser.setRTS(False)
            self._ser.setDTR(False)
        except OSError:
            pass  # the port has no RTS/DTR lines (e.g. a virtual port)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def send(self, cmd):
        with self._lock:
            self._ser.write((_encode(cmd) + "\n").encode())

    def _read_loop(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            msg = _parse_feedback(raw.decode(errors="ignore"))
            if msg is not None:
                self.feedback = msg
                self.feedback_event.set()

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its pose (T:105). Returns the T:1051 dict or None."""
        with self._request_lock:
            self.feedback_event.clear()
            self.send({"T": 105})
            if self.feedback_event.wait(timeout):
                return self.feedback
            return None

    def close(self):
        self._running = False
        self._ser.close()


# ---------------------------------------------------------------------------
# Recordings
# ---------------------------------------------------------------------------
# A recording is every command sent while recording, with its time in seconds
# from the start of the recording, plus the gripper tip pose when it started.

def _pose_from_fields(d, keys=("x", "y", "z", "t", "r", "g")):
    """Pose from a dict, kept inside the workspace and angle ranges. Raises ValueError."""
    values = [float(d[k]) for k in keys]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("a value is not a number")
    pose = Pose(*values)
    pose.x, pose.y, pose.z = clamp_position(pose.x, pose.y, pose.z)
    for name, (lo, hi) in ANGLE_RANGES.items():
        setattr(pose, name, clamp(getattr(pose, name), lo, hi))
    return pose


def _check_command(cmd):
    """Return a clean copy of a recorded command, or raise ValueError."""
    kind = cmd["T"]
    if kind == 1041:
        return _pose_from_fields(cmd).command()
    if kind == 100:
        return {"T": 100}
    raise ValueError(f"unsupported command T:{kind}")


def save_recording(path, data):
    """Write a recording as JSON, one command per line so the file stays readable."""
    head = {k: v for k, v in data.items() if k != "commands"}
    lines = [json.dumps(c, separators=(",", ":")) for c in data["commands"]]
    text = (json.dumps(head, indent=2)[:-2]
            + ',\n  "commands": [\n    ' + ",\n    ".join(lines) + "\n  ]\n}\n")
    with open(path, "w") as f:
        f.write(text)


def load_recording(path):
    """Read and check a recording file. Raises OSError or ValueError."""
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, dict) or data.get("format") != RECORDING_FORMAT:
        raise ValueError("not a RoArm-M3 Quest 2 recording")
    if data.get("version") != RECORDING_VERSION:
        raise ValueError(f"unsupported recording version {data.get('version')!r}")
    try:
        start = _pose_from_fields(data["start"])
        commands = [{"t": max(0.0, float(item["t"])), "cmd": _check_command(item["cmd"])}
                    for item in data["commands"]]
        duration = float(data["duration"])
    except (KeyError, TypeError, AttributeError) as e:
        raise ValueError(f"missing or invalid field ({e})")
    if not commands:
        raise ValueError("the recording is empty")
    commands.sort(key=lambda c: c["t"])
    return {"start": start, "duration": max(duration, commands[-1]["t"]), "commands": commands}


def is_recording_name(name):
    """True if name is a plain .json file name (no folders), so it can't point outside recordings/."""
    return (isinstance(name, str) and name == Path(name).name and name.endswith(".json")
            and not name.startswith("."))


def list_recordings():
    """Recordings in RECORDINGS_DIR, newest first, with their length or the reason they can't play."""
    items = []
    paths = sorted(RECORDINGS_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True) \
        if RECORDINGS_DIR.is_dir() else []
    for path in paths:
        item = {"name": path.name}
        try:
            rec = load_recording(path)
            item.update(duration=round(rec["duration"], 1), commands=len(rec["commands"]))
        except (OSError, ValueError) as e:
            item["error"] = str(e)
        items.append(item)
    return items


def new_recording_path():
    RECORDINGS_DIR.mkdir(exist_ok=True)
    base = time.strftime("recording-%Y%m%d-%H%M%S")
    path, n = RECORDINGS_DIR / f"{base}.json", 2
    while path.exists():
        path, n = RECORDINGS_DIR / f"{base}-{n}.json", n + 1
    return path


def _cylindrical(p):
    return math.hypot(p.x, p.y), math.atan2(p.y, p.x), p.z


def glide_time(a, b):
    """Seconds to glide from pose a to pose b within the PLAYBACK speeds."""
    r1, th1, z1 = _cylindrical(a)
    r2, th2, z2 = _cylindrical(b)
    path = math.sqrt((r2 - r1) ** 2 + ((r1 + r2) / 2 * (th2 - th1)) ** 2 + (z2 - z1) ** 2)
    turn = max(abs(b.t - a.t), abs(b.r - a.r), abs(b.g - a.g))
    # x1.5: the smoothstep in glide_pose() peaks at 1.5x its average speed.
    return 1.5 * max(path / PLAYBACK_LINEAR_SPEED, turn / PLAYBACK_ANGULAR_SPEED)


def glide_pose(a, b, s):
    """Pose a fraction s (0..1) of the way from a to b, easing in and out.
    Moves around the base (not straight across), so it never passes close to the
    base axis when both ends are inside the workspace."""
    s = s * s * (3 - 2 * s)
    r1, th1, z1 = _cylindrical(a)
    r2, th2, z2 = _cylindrical(b)
    r, th = r1 + (r2 - r1) * s, th1 + (th2 - th1) * s
    return Pose(r * math.cos(th), r * math.sin(th), z1 + (z2 - z1) * s,
                a.t + (b.t - a.t) * s, a.r + (b.r - a.r) * s, a.g + (b.g - a.g) * s)


@dataclass
class Playback:
    rec: dict
    name: str
    loops: int                 # 0 = forever
    loop: int = 1
    phase: str = "reading"     # reading -> to_start -> settle -> play
    phase_t0: float = 0.0
    index: int = 0             # next command to play
    glide_from: Pose = None
    glide_time: float = 0.0
    goal: Pose = None          # latest recorded T:1041 target


# ---------------------------------------------------------------------------
# Teleoperation
# ---------------------------------------------------------------------------
NO_FEEDBACK = "No feedback from the arm, so its position is unknown. Check the serial connection."
BUTTON_LABELS = {"right": ("A", "B"), "left": ("X", "Y")}  # (lower, upper) button


class Teleop:
    """Turns controller updates into T:1041 commands, and records and plays them back."""

    def __init__(self, link, hand="right", scale=DEFAULT_SCALE, arm_yaw=0.0, gripper_mode="analog"):
        self.link = link
        self.hand_name = hand
        self.other_name = "left" if hand == "right" else "right"
        self.scale = scale
        self.arm_yaw = math.radians(arm_yaw)
        self.gripper_mode = gripper_mode
        self.target = HOME_POSE.copy()   # pose the arm is being sent to
        self.last_sent = self.target.copy()
        self.last_send_time = 0.0
        self.commanded = False           # target was sent by us (not just read back)
        self.input = None                # latest message from the headset
        self.input_time = 0.0
        self.prev_buttons = {self.hand_name: set(), self.other_name: set()}
        self.engaged = False
        self.engage_blocked = False      # wait for the grip to be released first
        self.read_future = None          # arm position being read in the background
        self.read_purpose = None         # what the reading is for: "engage", "record" or "play"
        self.anchor = None
        self.filtered = None             # smoothed desired pose
        self.follow_orientation = True
        self.gripper_latched = False
        self.gripper_goal = self.target.g
        self.takeover_side = 1.0
        self.limited = False
        self.next_limit_haptic = 0.0
        self.home_until = 0.0
        self.status = ""
        self.haptics = []                # (hand, intensity, ms) waiting to go to the headset
        self.recording = None            # dict while recording
        self.playback = None             # Playback while playing
        items = list_recordings()
        self.selected_name = next((i["name"] for i in items if "error" not in i), None)
        self.selected_loops = 1
        self.recordings_changed = True   # send the list of recordings to the pages

    # -- helpers ----------------------------------------------------------
    def say(self, text):
        if text != self.status:
            self.status = text
            print(time.strftime("%H:%M:%S"), text, flush=True)

    def haptic(self, pulse, hand=None):
        self.haptics.append((hand or self.hand_name, *pulse))

    def _send(self, cmd):
        try:
            self.link.send(cmd)
        except Exception as e:  # e.g. USB cable pulled out
            self.say(f"Serial write failed: {e}")
        if self.recording is not None:
            t = round(time.monotonic() - self.recording["t0"], 3)
            self.recording["commands"].append({"t": t, "cmd": cmd})

    def sync_from_arm(self):
        """Read the arm's pose at startup. Returns True if it answered."""
        fb = self.link.request_feedback()
        if fb is None:
            return False
        self.target = pose_from_feedback(fb, HOME_POSE)
        self.last_sent = self.target.copy()
        return True

    # -- reading the arm's position in the background ----------------------
    def _start_read(self, purpose):
        # Runs in a thread so the server keeps going while it waits for the arm.
        self.read_future = asyncio.get_running_loop().run_in_executor(None, self.link.request_feedback)
        self.read_purpose = purpose

    def _cancel_read(self, purpose):
        if self.read_purpose == purpose:
            self.read_future = self.read_purpose = None

    def _take_read(self):
        """Pose from a finished read, or None if the arm didn't answer.

        The reported position is used unless it is close to the last commanded
        target, in which case the target is kept (see RESYNC_TOLERANCE_MM)."""
        future = self.read_future
        self.read_future = self.read_purpose = None
        fb = future.result()
        if fb is None:
            return None
        measured = pose_from_feedback(fb, self.target)
        if self.commanded and measured.close_to(self.target):
            return self.target.copy()
        if self.commanded:
            measured.g = self.target.g  # keep squeezing a held object
        return measured

    def _read_done(self, purpose):
        return self.read_purpose == purpose and self.read_future.done()

    # -- input from the pages -----------------------------------------------
    def on_message(self, data, now):
        """Controller updates from the headset that is in control."""
        kind = data.get("type")
        if kind == "input":
            self.input = data
            self.input_time = now
        elif kind == "session" and data.get("state") == "end":
            self.input = None

    def on_command(self, data, now):
        """Recording and playback commands from any page."""
        kind = data.get("type")
        if kind == "select":
            self.select(data.get("name"), data.get("loops"))
        elif kind == "record":
            self.toggle_recording(now)
        elif kind == "play":
            name = data.get("name")
            if name is not None and not is_recording_name(name):
                self.say(f"Can't play {name!r}: not a recording in the recordings folder.")
                return
            self.select(name, data.get("loops"))
            self.start_playback(now)
        elif kind == "stop":
            if self.playback:
                self.stop_playback(now, "Playback stopped.")
        elif kind == "list":
            self.recordings_changed = True

    def select(self, name, loops):
        if name is not None and is_recording_name(name):
            self.selected_name = name
        if loops is not None:
            try:
                self.selected_loops = max(0, min(999999, int(loops)))
            except (TypeError, ValueError):
                pass

    def client_gone(self):
        self.input = None

    def _fresh_input(self, now):
        if self.input is None or now - self.input_time > POSE_TIMEOUT:
            return None
        hands = self.input.get("hands")
        return hands if isinstance(hands, dict) else None

    def _hand(self, now):
        """The controlled hand from the latest update, or None if it is stale or missing."""
        hands = self._fresh_input(now)
        if hands is None or not isinstance(hands.get(self.hand_name), dict):
            return None
        return parse_hand(hands[self.hand_name])

    def _other_buttons(self, now):
        """Buttons held on the other controller (record / play)."""
        hands = self._fresh_input(now)
        other = hands.get(self.other_name) if hands else None
        if not isinstance(other, dict):
            return set()
        return {k for k in ("a", "b") if other.get(k) is True}

    def _head_quat(self):
        try:
            return _floats(self.input["head"]["quat"], 4)
        except (KeyError, TypeError, ValueError):
            return (0.0, 0.0, 0.0, 1.0)

    def _pressed(self, hand_name, down):
        """Buttons that went down since the last update."""
        pressed = down - self.prev_buttons[hand_name]
        self.prev_buttons[hand_name] = down
        return pressed

    def _hand_buttons(self, hand):
        if hand is None:
            return set()
        down = {k for k in ("a", "b") if getattr(hand, k)}
        # The trigger counts as pressed above 0.6 and released below 0.4.
        if hand.trigger > 0.6 or ("trigger" in self.prev_buttons[self.hand_name] and hand.trigger > 0.4):
            down.add("trigger")
        return down

    # -- main update ------------------------------------------------------
    def update(self, now, dt):
        hand = self._hand(now)
        pressed = self._pressed(self.hand_name, self._hand_buttons(hand))
        other = self._pressed(self.other_name, self._other_buttons(now))

        if self._read_done("record"):
            self._begin_recording(now, self._take_read())
        if self._read_done("play"):
            self._playback_position(now, self._take_read())
        if "a" in other:
            self.toggle_recording(now)
        if "b" in other:
            if self.playback:
                self.stop_playback(now, "Playback stopped.")
            else:
                self.start_playback(now)

        if self.playback:
            # The controls are ignored while playing, but squeezing the grip takes over.
            if hand is not None and hand.grip >= GRIP_ON:
                self.stop_playback(now, "Playback stopped: you took over with the grip.")
            else:
                self._update_playback(now, dt)
            self._send_target(now)
            return

        if "a" in pressed:
            self.go_home(now)
        if "b" in pressed:
            self.follow_orientation = not self.follow_orientation
            self.haptic(HAPTIC_TICK)
            self.say("Following pitch and roll." if self.follow_orientation
                     else "Pitch and roll locked; following position only.")
            if self.engaged:
                self._anchor(hand, self.target)  # carry on from here without a jump

        threshold = GRIP_OFF if (self.engaged or self.read_purpose == "engage") else GRIP_ON
        grip_held = hand is not None and hand.grip >= threshold

        if not grip_held:
            if self.engaged:
                self.disengage(now, "Tracking lost: stopped following." if hand is None
                               else "Released: the arm holds its position.")
                if hand is None:
                    self.engage_blocked = True  # squeeze again once tracking is back
                else:
                    self.haptic(HAPTIC_RELEASE)
            self._cancel_read("engage")
            if hand is not None:
                self.engage_blocked = False
        elif not self.engaged and not self.engage_blocked:
            self._try_engage(hand, now)

        if self.engaged:
            self._follow(hand, now, dt, pressed)
        self._send_target(now)

    # -- engaging -----------------------------------------------------------
    def _refuse(self, text, hand=None):
        if hand is None:
            self.engage_blocked = True
        self.haptic(HAPTIC_ERROR, hand)
        self.say(text)

    def _try_engage(self, hand, now):
        if now < self.home_until:
            self._refuse("Wait: the arm is moving to its home position.")
            return
        if self.link.simulated:
            self._engage(hand, self.target)
            return
        if self.read_purpose == "engage":
            if not self.read_future.done():
                return
            start = self._take_read()
            if start is None:
                self._refuse(NO_FEEDBACK)
            else:
                self._engage(hand, start)
            return
        if self.read_purpose is None:  # otherwise wait for the other read to finish
            # Read where the arm really is, in case something else moved it.
            self._start_read("engage")
            self.say("Reading the arm's position...")

    def _engage(self, hand, start):
        problem = outside_workspace(start)
        if problem:
            self._refuse(f"Can't follow: {problem}. Press {BUTTON_LABELS[self.hand_name][0]} "
                         "to send the arm home, "
                         "or widen the limits in quest2_controller.py.")
            return
        self.target = start.copy()
        self.last_sent = start.copy()
        self.filtered = start.copy()
        self._anchor(hand, start)
        self.engaged = True
        self.limited = False
        trigger_g = self._trigger_gripper(hand)
        self.gripper_latched = self.gripper_mode == "analog" and abs(trigger_g - start.g) <= GRIPPER_TAKEOVER
        # Which side of the gripper's angle the trigger starts on, to spot it crossing over.
        self.takeover_side = 1.0 if trigger_g > start.g else -1.0
        self.gripper_goal = start.g  # toggle mode: the angle the gripper is heading for
        self.haptic(HAPTIC_ENGAGE)
        msg = "Following the controller."
        if self.gripper_mode == "analog" and not self.gripper_latched:
            msg += (" The gripper stays as it is until the trigger reaches its position "
                    f"({'pull' if self.takeover_side < 0 else 'release'} the trigger).")
        self.say(msg)

    def _anchor(self, hand, arm):
        pitch, roll = pitch_roll(hand.quat)
        forward, right = horizontal_frame(self._head_quat())
        self.anchor = Anchor(hand.pos, forward, right, pitch, 0.0, roll, arm.copy())
        self.filtered = arm.copy()

    def disengage(self, now, text):
        self.engaged = False
        self.anchor = None
        self.filtered = None
        self.limited = False
        self._send_target(now, force=True)  # finish exactly where the target is
        self.say(text)

    def go_home(self, now):
        if self.engaged:
            self.disengage(now, "Moving to home position.")
        self.engage_blocked = True
        self._cancel_read("engage")
        self._send({"T": 100})
        self.target = HOME_POSE.copy()
        self.last_sent = self.target.copy()
        self.commanded = False  # read the real position on the next squeeze
        self.home_until = now + HOME_WAIT
        self.say("Moving to home position.")

    # -- following ----------------------------------------------------------
    @staticmethod
    def _trigger_gripper(hand):
        return GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * hand.trigger

    def _desired(self, hand, pressed):
        """Arm pose that matches the controller, before limits and smoothing."""
        a = self.anchor
        d = [hand.pos[i] - a.hand_pos[i] for i in range(3)]
        k = 1000.0 * self.scale  # m -> mm
        ahead = (d[0] * a.forward[0] + d[2] * a.forward[2]) * k
        left = -(d[0] * a.right[0] + d[2] * a.right[2]) * k
        up = d[1] * k
        # Turn "ahead/left of you" into the arm's X/Y (see --arm-yaw).
        c, s = math.cos(self.arm_yaw), math.sin(self.arm_yaw)
        desired = Pose(a.arm.x + ahead * c + left * s,
                       a.arm.y - ahead * s + left * c,
                       a.arm.z + up,
                       a.arm.t, a.arm.r, self.target.g)
        if self.follow_orientation:
            pitch, roll = pitch_roll(hand.quat)
            a.roll += wrap_angle(roll - a.last_roll)  # unwrapped, so it can turn past 180 deg
            a.last_roll = roll
            desired.t = a.arm.t + PITCH_SIGN * (pitch - a.pitch)
            desired.r = a.arm.r + ROLL_SIGN * a.roll
        if self.gripper_mode == "analog":
            g = self._trigger_gripper(hand)
            gap = g - self.target.g
            if not self.gripper_latched and (abs(gap) <= GRIPPER_TAKEOVER or gap * self.takeover_side <= 0):
                self.gripper_latched = True
                self.haptic(HAPTIC_TICK)
                self.say("Following the controller. The trigger now controls the gripper.")
            if self.gripper_latched:
                desired.g = g
        else:
            if "trigger" in pressed:
                closed = self.gripper_goal > (GRIPPER_OPEN + GRIPPER_CLOSED) / 2
                self.gripper_goal = GRIPPER_OPEN if closed else GRIPPER_CLOSED
            desired.g = self.gripper_goal
        return desired

    def _follow(self, hand, now, dt, pressed):
        desired = self._desired(hand, pressed)

        limited = past_limit(desired)
        if limited and (not self.limited or now >= self.next_limit_haptic):
            self.haptic(HAPTIC_LIMIT)
            self.next_limit_haptic = now + HAPTIC_LIMIT_EVERY
        if limited and not self.limited:
            self.say("At a limit: the arm stops at the edge of its workspace.")
        elif self.limited and not limited:
            self.say("Following the controller.")
        self.limited = limited

        # Smooth out hand tremor.
        alpha = 1.0 - math.exp(-dt / SMOOTHING) if SMOOTHING > 0 else 1.0
        f = self.filtered
        for name in ("x", "y", "z", "t", "r", "g"):
            cur = getattr(f, name)
            setattr(f, name, cur + (getattr(desired, name) - cur) * alpha)
        self._step_toward(f, dt)

    def _step_toward(self, goal, dt):
        """Move the target toward goal at a limited speed, staying inside the limits.
        Small steps also keep clamp_position() from jumping across the base."""
        t = self.target
        dx, dy, dz = goal.x - t.x, goal.y - t.y, goal.z - t.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        max_step = MAX_LINEAR_SPEED * dt
        if dist > max_step:
            dx, dy, dz = (v * max_step / dist for v in (dx, dy, dz))
        t.x, t.y, t.z = clamp_position(t.x + dx, t.y + dy, t.z + dz)
        max_turn = MAX_ANGULAR_SPEED * dt
        for name, (lo, hi) in ANGLE_RANGES.items():
            old = getattr(t, name)
            new = old + clamp(getattr(goal, name) - old, -max_turn, max_turn)
            setattr(t, name, step_within_range(old, new, lo, hi))

    def _send_target(self, now, force=False):
        """Send the target with T:1041 if it changed (at most SEND_HZ per second)."""
        if not self.target.differs(self.last_sent):
            return
        if not force and now - self.last_send_time < 1.0 / SEND_HZ:
            return
        self._send(self.target.command())
        self.last_sent = self.target.copy()
        self.last_send_time = now
        self.commanded = True

    # -- recording --------------------------------------------------------
    def toggle_recording(self, now):
        record_key = BUTTON_LABELS[self.other_name][0]
        if self.recording is not None:
            self.finish_recording(now)
            return
        if self.read_purpose == "record":
            self._cancel_read("record")
            self.say("Recording cancelled.")
            return
        if self.playback:
            self._refuse("Stop the playback before recording.", self.other_name)
            return
        if self.engaged or self.link.simulated:
            self._begin_recording(now, self.target.copy())
        else:
            # The start position must be where the arm really is.
            self._start_read("record")
            self.say(f"Reading the arm's position before recording ({record_key} = cancel)...")

    def _begin_recording(self, now, pose):
        if pose is None:
            self._refuse(f"Can't start recording. {NO_FEEDBACK}", self.other_name)
            return
        if not self.engaged:
            self.target = pose.copy()
            self.last_sent = pose.copy()
        self.recording = {"t0": now, "start": pose.copy(), "commands": []}
        self.haptic(HAPTIC_RECORD, self.other_name)
        self.say(f"Recording. Press {BUTTON_LABELS[self.other_name][0]} again to stop and save.")

    def finish_recording(self, now):
        """Stop recording and save it in RECORDINGS_DIR. Returns the file name or None."""
        self._send_target(now, force=True)  # so the final position is recorded
        rec, self.recording = self.recording, None
        self.haptic(HAPTIC_RECORD, self.other_name)
        if not rec["commands"]:
            self.say("Recording stopped: nothing was recorded.")
            return None
        data = {
            "format": RECORDING_FORMAT,
            "version": RECORDING_VERSION,
            "duration": round(now - rec["t0"], 3),
            "start": {k: round(getattr(rec["start"], k), 1 if k in "xyz" else 4) for k in "xyztrg"},
            "commands": rec["commands"],
        }
        path = new_recording_path()
        try:
            save_recording(path, data)
        except OSError as e:
            self.say(f"Could not save the recording: {e.strerror}.")
            return None
        self.selected_name = path.name
        self.recordings_changed = True
        self.say(f"Saved {path.name} ({data['duration']:.1f} s, {len(data['commands'])} commands).")
        return path.name

    # -- playback ---------------------------------------------------------
    def start_playback(self, now):
        if self.recording is not None:
            self._refuse("Stop recording before playing a recording.", self.other_name)
            return
        name = self.selected_name
        if not name:
            self._refuse("No recording to play. Record one first.", self.other_name)
            return
        try:
            rec = load_recording(RECORDINGS_DIR / name)
        except (OSError, ValueError) as e:
            self._refuse(f"Can't play {name}: {e}", self.other_name)
            return
        if self.engaged:
            self.disengage(now, "Playing a recording.")
        self._cancel_read("engage")
        self.engage_blocked = True  # release the grip before following again
        self.playback = Playback(rec, name, self.selected_loops)
        self.haptic(HAPTIC_PLAY, self.other_name)
        if self.link.simulated:
            self._playback_position(now, self.target.copy())
        else:
            # The glide to the start position begins where the arm really is.
            self._start_read("play")
            self.say(f"Playing {name}: reading the arm's position...")

    def _playback_position(self, now, pose):
        """The arm's position has been read: start gliding to the recording's start."""
        if self.playback is None:
            return
        if pose is None:
            self.playback = None
            self._refuse(f"Playback stopped. {NO_FEEDBACK}", self.other_name)
            return
        problem = outside_workspace(pose)
        if problem:
            self.playback = None
            self._refuse(f"Can't play: {problem}. Press {BUTTON_LABELS[self.hand_name][0]} "
                         "to send the arm home first.", self.other_name)
            return
        self.target = pose.copy()
        self.last_sent = pose.copy()
        self._begin_loop(now)

    def _begin_loop(self, now):
        pb = self.playback
        pb.glide_from = self.target.copy()
        pb.glide_time = glide_time(pb.glide_from, pb.rec["start"])
        pb.phase, pb.phase_t0, pb.goal = "to_start", now, None
        total = "forever" if pb.loops == 0 else f"of {pb.loops}"
        self.say(f"Playing {pb.name}, loop {pb.loop} {total}.")

    def stop_playback(self, now, text):
        """Interrupt playback. The arm stops after its current small step."""
        self._cancel_read("play")
        self.playback = None
        self._send_target(now, force=True)
        self.haptic(HAPTIC_PLAY, self.other_name)
        self.say(text)

    def _update_playback(self, now, dt):
        pb = self.playback
        if pb.phase == "reading":
            return
        if pb.phase == "to_start":
            s = 1.0 if pb.glide_time <= 0 else min(1.0, (now - pb.phase_t0) / pb.glide_time)
            self.target = glide_pose(pb.glide_from, pb.rec["start"], s)
            if s >= 1.0:
                self._send_target(now, force=True)
                pb.phase, pb.phase_t0 = "settle", now
        elif pb.phase == "settle":
            if now - pb.phase_t0 >= PLAYBACK_SETTLE:
                pb.phase, pb.phase_t0, pb.index = "play", now, 0
        else:
            elapsed = now - pb.phase_t0
            commands = pb.rec["commands"]
            while pb.index < len(commands) and commands[pb.index]["t"] <= elapsed:
                self._play_command(commands[pb.index]["cmd"], now)
                pb.index += 1
            if pb.goal is not None:
                # Through the speed limit and workspace limits, so even an edited
                # file can't make the arm jump.
                self._step_toward(pb.goal, dt)
            if pb.index >= len(commands) and elapsed >= pb.rec["duration"]:
                if pb.loops and pb.loop >= pb.loops:
                    self.playback = None
                    self._send_target(now, force=True)
                    self.haptic(HAPTIC_PLAY, self.other_name)
                    self.say(f"Finished playing {pb.name}.")
                else:
                    pb.loop += 1
                    self._begin_loop(now)

    def _play_command(self, cmd, now):
        if cmd["T"] == 1041:
            self.playback.goal = _pose_from_fields(cmd)
        elif cmd["T"] == 100:
            self._send({"T": 100})
            self.target = HOME_POSE.copy()
            self.last_sent = self.target.copy()
            self.playback.goal = None

    # -- status -----------------------------------------------------------
    def snapshot(self, now):
        """State shown on the pages."""
        recording = None
        if self.recording is not None:
            recording = {"elapsed": round(now - self.recording["t0"], 1),
                         "commands": len(self.recording["commands"])}
        playback = None
        if self.playback is not None:
            pb = self.playback
            playback = {"name": pb.name, "loop": pb.loop, "loops": pb.loops, "phase": pb.phase,
                        "elapsed": round(min(now - pb.phase_t0, pb.rec["duration"]), 1) if pb.phase == "play" else 0,
                        "duration": round(pb.rec["duration"], 1)}
        return {
            "type": "status",
            "link": self.link.label,
            "hand": self.hand_name,
            "otherHand": self.other_name,
            "buttons": {"home": BUTTON_LABELS[self.hand_name][0], "orientation": BUTTON_LABELS[self.hand_name][1],
                        "record": BUTTON_LABELS[self.other_name][0], "play": BUTTON_LABELS[self.other_name][1]},
            "engaged": self.engaged,
            "limited": self.limited,
            "syncing": self.read_purpose is not None,
            "orientation": self.follow_orientation,
            "gripperMode": self.gripper_mode,
            "gripperLatched": self.gripper_latched,
            "target": self.target.as_dict(),
            "recording": recording,
            "playback": playback,
            "selected": {"name": self.selected_name, "loops": self.selected_loops},
            "message": self.status,
        }


# ---------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------
class Server:
    """Serves the page, and talks to every open page over a WebSocket.

    Any number of pages can be open (e.g. the headset and a browser on the
    computer). All of them see the status and can record and play. Only one
    sends controller input: the headset that most recently started its
    immersive view."""

    STATUS_HZ = 10

    def __init__(self, teleop):
        self.teleop = teleop
        self.clients = {}       # WebSocket -> {"task": send in progress, "list": list still to send}
        self.controller = None  # the WebSocket whose controller input is used

    def make_app(self):
        app = web.Application()
        app.router.add_get("/", self.index)
        app.router.add_get("/ws", self.websocket)
        app.router.add_static("/static/", WEB_DIR)
        return app

    async def index(self, request):
        return web.FileResponse(WEB_DIR / "index.html", headers={"Cache-Control": "no-store"})

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        self.clients[ws] = {"task": None, "list": True}
        self.teleop.say(f"Browser connected from {request.remote}.")
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except ValueError:
                    continue
                if isinstance(data, dict):
                    self._handle(ws, data, time.monotonic())
        finally:
            del self.clients[ws]
            if self.controller is ws:
                self.controller = None
                self.teleop.client_gone()
                self.teleop.say("Headset disconnected.")
        return ws

    def _handle(self, ws, data, now):
        kind = data.get("type")
        if kind == "session" and data.get("state") == "start":
            if self.controller is not None and self.controller is not ws:
                # The newest headset takes over.
                self.teleop.client_gone()
                self._post(self.controller, [{"type": "notice", "message": "Another headset took over control."}])
            self.controller = ws
        if kind in ("input", "session"):
            if kind == "input" and self.controller is None:
                self.controller = ws  # e.g. the page reconnected after a server restart
            if ws is self.controller:
                self.teleop.on_message(data, now)
                if kind == "session" and data.get("state") == "end":
                    self.controller = None
        else:
            self.teleop.on_command(data, now)

    def _post(self, ws, messages):
        """Send messages unless the last send to ws is still going (a slow page
        must never hold up the arm). Returns False if they were dropped."""
        state = self.clients.get(ws)
        if state is None or ws.closed or (state["task"] and not state["task"].done()):
            return False
        state["task"] = asyncio.create_task(self._send(ws, messages))
        return True

    @staticmethod
    async def _send(ws, messages):
        try:
            await ws.send_json(messages)
        except (ConnectionError, RuntimeError):
            pass  # the page went away; websocket() cleans up

    async def control_loop(self):
        period = 1.0 / LOOP_HZ
        last = time.monotonic()
        next_status = 0.0
        while True:
            now = time.monotonic()
            dt = min(now - last, 0.1)  # don't jump after a stall
            last = now
            self.teleop.update(now, dt)

            if self.teleop.recordings_changed:
                self.teleop.recordings_changed = False
                for state in self.clients.values():
                    state["list"] = True
            send_status = now >= next_status
            if send_status:
                next_status = now + 1.0 / self.STATUS_HZ
                status = self.teleop.snapshot(now)
            items = None
            for ws, state in list(self.clients.items()):
                messages = []
                if ws is self.controller:
                    messages += [{"type": "haptic", "hand": hand, "intensity": i, "ms": ms}
                                 for hand, i, ms in self.teleop.haptics]
                if state["list"]:
                    if items is None:
                        items = list_recordings()
                    messages.append({"type": "recordings", "items": items})
                if send_status:
                    messages.append(status)
                if messages and self._post(ws, messages):
                    state["list"] = False
            self.teleop.haptics.clear()
            await asyncio.sleep(max(0.0, period - (time.monotonic() - now)))


# ---------------------------------------------------------------------------
# HTTPS certificate
# ---------------------------------------------------------------------------
# WebXR only works on secure (https) pages, so the server needs a certificate.
# A self-signed one is made on first run. The Quest Browser warns about it
# once; after you accept it, WebXR works.

def local_addresses():
    """IPv4 addresses other devices can probably reach this computer on."""
    found = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packet is sent
            found.append(s.getsockname()[0])
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            found.append(info[4][0])
    except OSError:
        pass
    out = []
    for ip in found:
        if ip not in out and not ip.startswith("127."):
            out.append(ip)
    return out


def ensure_certificate(addresses):
    """Path of a self-signed certificate and key covering addresses, made if needed."""
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

    cert_path, key_path = CERT_DIR / "server.crt", CERT_DIR / "server.key"
    wanted = {ipaddress.ip_address(a) for a in addresses + ["127.0.0.1"]}
    if cert_path.exists() and key_path.exists():
        try:
            cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
            san = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
            fresh = cert.not_valid_after_utc > datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(days=1)
            if fresh and wanted <= set(san.get_values_for_type(x509.IPAddress)):
                return cert_path, key_path
        except (ValueError, x509.ExtensionNotFound):
            pass
        print("Making a new certificate (this computer's address changed). "
              "The Quest Browser will ask you to accept it again.")

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "RoArm-M3 Quest 2 controller")])
    now = datetime.datetime.now(datetime.timezone.utc)
    hostnames = {"localhost", socket.gethostname()}
    san = [x509.DNSName(h) for h in sorted(hostnames)] + [x509.IPAddress(ip) for ip in sorted(wanted)]
    cert = (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(minutes=5))
            .not_valid_after(now + datetime.timedelta(days=825))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .sign(key, hashes.SHA256()))
    CERT_DIR.mkdir(exist_ok=True)
    key_path.write_bytes(key.private_bytes(serialization.Encoding.PEM,
                                           serialization.PrivateFormat.TraditionalOpenSSL,
                                           serialization.NoEncryption()))
    key_path.chmod(0o600)
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    return cert_path, key_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
async def run(args, link):
    teleop = Teleop(link, hand=args.hand, scale=args.scale, arm_yaw=args.arm_yaw,
                    gripper_mode=args.gripper)
    if link.simulated:
        teleop.say("Dry run: commands are printed to the terminal.")
    elif teleop.sync_from_arm():
        t = teleop.target
        teleop.say(f"Arm found: tip at x={t.x:.0f} y={t.y:.0f} z={t.z:.0f} mm.")
    else:
        teleop.say("The arm didn't answer yet. It will be asked again when you squeeze the grip.")

    ssl_ctx = None
    if not args.no_tls:
        if args.cert:
            cert_path, key_path = args.cert, args.key
        else:
            cert_path, key_path = ensure_certificate(local_addresses())
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(cert_path, key_path)

    server = Server(teleop)
    runner = web.AppRunner(server.make_app(), access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.listen_port, ssl_context=ssl_ctx)
    try:
        await site.start()
    except OSError as e:
        await runner.cleanup()
        sys.exit(f"Can't listen on port {args.listen_port}: {e.strerror}")

    scheme = "http" if args.no_tls else "https"
    print()
    print("Open this address in the Quest Browser:")
    if args.no_tls:
        print(f"  {scheme}://localhost:{args.listen_port}   (after: adb reverse tcp:{args.listen_port} tcp:{args.listen_port})")
    else:
        for ip in local_addresses() or ["<this computer's IP>"]:
            print(f"  {scheme}://{ip}:{args.listen_port}")
    print("Press Ctrl+C to quit.")
    print()
    # Shut down cleanly (saving any recording) on `kill` as well as Ctrl+C.
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, asyncio.current_task().cancel)
    try:
        await server.control_loop()
    except asyncio.CancelledError:
        pass
    finally:
        if teleop.recording is not None:
            teleop.finish_recording(time.monotonic())  # don't lose it on quit
        await runner.cleanup()


def main():
    sys.stdout.reconfigure(line_buffering=True)  # show messages right away, even when piped
    parser =argparse.ArgumentParser(description="Meta Quest 2 controller for the Waveshare RoArm-M3")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port of the arm (default /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    parser.add_argument("--dry-run", action="store_true", help="don't connect to the arm; print commands instead")
    parser.add_argument("--hand", choices=("right", "left"), default="right",
                        help="controller that moves the arm (default right)")
    parser.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                        help=f"arm mm per controller mm (default {DEFAULT_SCALE})")
    parser.add_argument("--arm-yaw", type=float, default=0.0, metavar="DEG",
                        help="which way the arm faces compared to you, in degrees counterclockwise "
                             "(0 = same way as you, 180 = facing you; default 0)")
    parser.add_argument("--gripper", choices=("analog", "toggle"), default="analog",
                        help="analog: the trigger sets how far the gripper closes; "
                             "toggle: each trigger pull opens or closes it (default analog)")
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on (default all)")
    parser.add_argument("--listen-port", type=int, default=DEFAULT_LISTEN_PORT,
                        help=f"port for the web page (default {DEFAULT_LISTEN_PORT})")
    parser.add_argument("--no-tls", action="store_true",
                        help="serve plain http (only for use with adb reverse and http://localhost)")
    parser.add_argument("--cert", help="certificate file to use instead of the self-signed one")
    parser.add_argument("--key", help="private key file for --cert")
    args = parser.parse_args()
    if bool(args.cert) != bool(args.key):
        parser.error("--cert and --key must be given together")
    if args.scale <= 0:
        parser.error("--scale must be greater than 0")

    link = DryRunLink() if args.dry_run else SerialLink(args.port, args.baud)
    if not args.dry_run:
        time.sleep(0.5)  # let the board settle after opening the port
    try:
        asyncio.run(run(args, link))
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        print("Bye.")


if __name__ == "__main__":
    main()
