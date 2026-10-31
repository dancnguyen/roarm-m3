#!/usr/bin/env python3
"""Keyboard controller for the Waveshare RoArm-M3 over serial or HTTP.

Two control modes (Tab switches between them):
- Joint mode: each key pair drives one joint with {"T":101} (radians).
- Axis mode: each key pair moves the gripper tip along one axis (X, Y, Z,
  pitch, roll, gripper) with {"T":123,"m":1}: the arm keeps moving the axis
  until it is told to stop.

- Joint mode: a tap moves the joint one small step. Holding the key moves it
  continuously, speeding up the longer the key is held (up to a maximum speed).
- Axis mode: the axis moves at a fixed speed while the key is held and stops
  when it is released. A tap gives a short pulse of motion.
- Each control's bar is also a slider: click or drag it with the mouse to
  send that joint or axis to the value under the mouse.
- Movements can be recorded to a JSON file and played back once, a number of
  times, or forever.

Keys are captured from a small pygame window (works on both X11 and Wayland),
so the window must have focus.

Usage:
    python keyboard_controller.py --port /dev/ttyUSB0
    python keyboard_controller.py --http 192.168.4.1   # over WiFi
    python keyboard_controller.py --dry-run            # no arm, print commands
"""

import argparse
import json
import math
import os
import string
import sys
import threading
import time
from dataclasses import dataclass

import pygame

try:
    import serial
except ImportError:  # only needed for serial mode
    serial = None

try:
    import requests
except ImportError:  # only needed for HTTP mode
    requests = None

try:
    import crossfiledialog
except Exception:  # not installed, or no zenity/kdialog on Linux
    crossfiledialog = None


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
# Joint mode
TAP_STEP = math.radians(2.0)     # rad moved by a quick tap
HOLD_DELAY = 0.25                # s before a held key starts continuous motion
START_SPEED = math.radians(10)   # rad/s when continuous motion begins
ACCELERATION = math.radians(40)  # rad/s^2 added while the key stays held
MAX_SPEED = math.radians(90)     # rad/s cap

LOOP_HZ = 50                     # control loop rate
SEND_HZ = 25                     # max T:101 commands per second per joint

# Passed through to T:101. spd = steps/s (0 = max), acc = 100 steps/s^2 (0 = max).
# Targets are updated in small increments, so letting the servo run at max
# speed between them gives the smoothest result.
CMD_SPD = 0
CMD_ACC = 0

# Axis mode (T:123 continuous movement)
# Speed coefficient passed to T:123. Larger is faster; the docs suggest 0-20.
AXIS_CMD_SPD = 5
AXIS_TAP_TIME = 0.1              # s an axis moves for a quick tap
AXIS_POLL_HZ = 10                # position reads (T:105) per second while an axis moves
AXIS_POLL_AFTER_STOP = 0.1       # s to wait after a stop before reading the final position

# Mouse sliders
# A joint moved with its slider glides to the slider value at up to MAX_SPEED.
# An axis moved with its slider runs (T:123) until the position read back is
# within this distance of the slider value, or has passed it.
SLIDER_TOLERANCE_MM = 2.0                # X/Y/Z
SLIDER_TOLERANCE_RAD = math.radians(1)   # pitch, roll, gripper
# Stop a slider-driven axis if no position has been read back for this long,
# since the controller can't tell when it reaches the slider value.
SLIDER_FEEDBACK_TIMEOUT = 0.5            # s

# T:114 LED brightness values for on and off.
LED_ON = 255
LED_OFF = 0

DEFAULT_HTTP_HOST = "192.168.4.1"  # the arm's address in WiFi AP mode
HTTP_TIMEOUT = 1.0                 # s to wait for each HTTP request

PLAYBACK_MOVE_SPEED = math.radians(45)  # rad/s when gliding to a recording's start position
PLAYBACK_SETTLE = 0.5                   # s to wait at the start position before playing

RECORDING_FORMAT = "roarm-m3-keyboard-recording"
RECORDING_VERSION = 1


@dataclass(eq=False)
class Control:
    """One key pair: a joint (T:101) or an axis of the gripper tip (T:123)."""
    mode: str          # "joint" or "axis"
    number: int        # joint id for T:101, axis id for T:123
    name: str
    minimum: float     # rad, or mm for linear axes
    maximum: float
    home: float        # value after T:100 (approximate for X/Y/Z)
    key_minus: int
    key_plus: int
    feedback_key: str  # field name in the T:1051 feedback message
    linear: bool = False  # True for X/Y/Z (mm)

    @property
    def unit(self):
        return "mm" if self.linear else "rad"

    def command(self, value):
        """T:101 command that moves a joint to value (rad)."""
        return {"T": 101, "joint": self.number, "rad": round(value, 4), "spd": CMD_SPD, "acc": CMD_ACC}

    def axis_command(self, direction, spd=AXIS_CMD_SPD):
        """T:123 command that starts moving an axis (+1 or -1) or stops it (0)."""
        return {"T": 123, "m": 1, "axis": self.number, "cmd": AXIS_CMD_OF[direction], "spd": spd}


# T:123 "cmd" values: 1 = increase, 2 = decrease, 0 = stop.
AXIS_CMD_OF = {+1: 1, -1: 2, 0: 0}
AXIS_DIRECTION_OF = {v: k for k, v in AXIS_CMD_OF.items()}


def is_axis_stop(cmd):
    return cmd.get("T") == 123 and cmd.get("cmd") == 0


# Joint ranges and home positions are from the Waveshare RoArm-M3 JSON command docs.
JOINTS = [
    Control("joint", 1, "Base",     -3.14,  3.14, 0.0,         pygame.K_a, pygame.K_q, "b"),
    Control("joint", 2, "Shoulder", -1.57,  1.57, 0.0,         pygame.K_s, pygame.K_w, "s"),
    Control("joint", 3, "Elbow",     0.0,   3.14, math.pi / 2, pygame.K_d, pygame.K_e, "e"),
    Control("joint", 4, "Wrist",    -1.57,  1.57, 0.0,         pygame.K_f, pygame.K_r, "t"),
    Control("joint", 5, "Roll",     -3.14,  3.14, 0.0,         pygame.K_g, pygame.K_t, "r"),
    Control("joint", 6, "Gripper",   1.08,  3.14, math.pi,     pygame.K_h, pygame.K_y, "g"),
]

# Axes for T:123 (m=1), using the same keys as the joints. X points straight
# ahead of the arm, Y to its left and Z up. The docs give no X/Y/Z limits, so
# these are cautious guesses: widen them once you know your arm's workspace.
# The controller stops an axis when the position it reads back reaches a limit.
# The X/Y/Z home values come from the feedback example in the docs and are only
# used when the arm can't report its position (e.g. --dry-run).
AXES = [
    Control("axis", 1, "X",        100.0, 450.0, 345.0,   pygame.K_a, pygame.K_q, "x", linear=True),
    Control("axis", 2, "Y",       -350.0, 350.0,   0.0,   pygame.K_s, pygame.K_w, "y", linear=True),
    Control("axis", 3, "Z",        -50.0, 450.0, 207.0,   pygame.K_d, pygame.K_e, "z", linear=True),
    Control("axis", 4, "Pitch",    -1.57,  1.57,   0.0,   pygame.K_f, pygame.K_r, "tit"),
    Control("axis", 5, "Roll",     -3.14,  3.14,   0.0,   pygame.K_g, pygame.K_t, "r"),
    Control("axis", 6, "Gripper",   1.08,  3.14, math.pi, pygame.K_h, pygame.K_y, "g"),
]

CONTROLS = {"joint": JOINTS, "axis": AXES}
JOINTS_BY_NUMBER = {c.number: c for c in JOINTS}
AXES_BY_NUMBER = {c.number: c for c in AXES}
MODE_LABEL = {"joint": "Joint mode (T:101)", "axis": "Axis mode (T:123)"}


def clamp(control, value):
    return max(control.minimum, min(control.maximum, value))


def step_within_range(control, value, delta):
    """Move value by delta without leaving the range. A value that is already
    outside the range (e.g. read from the arm) can move back in but not further out."""
    new = value + delta
    if delta > 0:
        return min(new, max(control.maximum, value))
    return max(new, min(control.minimum, value))


HOME_KEY = pygame.K_0
SYNC_KEY = pygame.K_p
LED_KEY = pygame.K_l
MODE_KEY = pygame.K_TAB
RECORD_KEY = pygame.K_m
PLAY_KEY = pygame.K_b
STOP_KEYS = (pygame.K_SPACE, pygame.K_ESCAPE)  # stop playback
QUIT_KEY = pygame.K_ESCAPE                      # when not playing


# ---------------------------------------------------------------------------
# Links to the arm
# ---------------------------------------------------------------------------
# Every link has the same interface: send(cmd), request_feedback() (waits for
# the answer), poll_feedback() + take_feedback() (ask now, collect the answer
# later without waiting), cancel_pending(), close(), an `error` attribute
# holding the last communication error (or None), and `simulated` (True only
# for --dry-run).


def _encode(cmd: dict) -> str:
    return json.dumps(cmd, separators=(",", ":"))


def _parse_feedback(text: str):
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

    error = None
    simulated = True

    def send(self, cmd: dict):
        print("TX", _encode(cmd))

    def request_feedback(self, timeout=1.0):
        return None

    def poll_feedback(self):
        pass

    def take_feedback(self):
        return None

    def cancel_pending(self):
        pass

    def close(self):
        pass


class _FeedbackStore:
    """Keeps the latest T:1051 feedback message for take_feedback()."""

    def _init_feedback(self):
        self.feedback = None          # latest T:1051 message (dict)
        self.feedback_event = threading.Event()
        self._feedback_seq = 0        # counts messages received
        self._taken_seq = 0

    def _store_feedback(self, msg):
        self.feedback = msg
        self._feedback_seq += 1
        self.feedback_event.set()

    def take_feedback(self):
        """Return feedback received since the last call, or None."""
        seq = self._feedback_seq
        if seq == self._taken_seq:
            return None
        self._taken_seq = seq
        return self.feedback


class SerialLink(_FeedbackStore):
    """Sends JSON commands over USB serial and reads feedback in a background thread."""

    simulated = False

    def __init__(self, port, baud):
        if serial is None:
            sys.exit("pyserial is not installed. Run: pip install -r requirements.txt")
        self.error = None
        self._init_feedback()
        self._lock = threading.Lock()
        self._running = True
        self._ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=None)
        # Avoid resetting the ESP32 when the port opens.
        self._ser.setRTS(False)
        self._ser.setDTR(False)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def send(self, cmd: dict):
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
                self._store_feedback(msg)

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its joint angles. Returns the dict or None."""
        self.feedback_event.clear()
        self.send({"T": 105})
        if self.feedback_event.wait(timeout):
            return self.feedback
        return None

    def poll_feedback(self):
        """Ask for feedback without waiting; the answer arrives via take_feedback()."""
        self.send({"T": 105})

    def cancel_pending(self):
        pass  # serial writes are sent immediately

    def close(self):
        self._running = False
        self._ser.close()


class HttpLink(_FeedbackStore):
    """Sends JSON commands as HTTP GET requests to http://<host>/js?json=<cmd>.

    Requests run in a background thread so a slow network never freezes the
    window. If commands are queued faster than the arm answers, an unsent
    command is replaced by a newer one of the same kind (the same joint for
    T:101, the same axis for T:123), so the arm always gets the latest target
    instead of a backlog. T:123 stop commands go to the front of the queue and
    are retried if a request fails, because the arm keeps moving until it gets one.
    """

    simulated = False
    STOP_ATTEMPTS = 3

    def __init__(self, host, timeout=HTTP_TIMEOUT):
        if requests is None:
            sys.exit("requests is not installed. Run: pip install -r requirements.txt")
        self.url = f"http://{host}/js"
        self.timeout = timeout
        self.error = None
        self._init_feedback()
        self._session = requests.Session()  # reuses the connection between requests
        self._session_lock = threading.Lock()
        self._pending = {}                  # key -> cmd, oldest first
        self._cond = threading.Condition()
        self._running = True
        self._worker = threading.Thread(target=self._send_loop, daemon=True)
        self._worker.start()

    def send(self, cmd: dict):
        key = (cmd.get("T"), cmd.get("joint", cmd.get("axis")))
        with self._cond:
            # Remove and re-add so the newest command goes to the back of the
            # queue, or to the front if it stops an axis.
            self._pending.pop(key, None)
            if is_axis_stop(cmd):
                self._pending = {key: cmd, **self._pending}
            else:
                self._pending[key] = cmd
            self._cond.notify()

    def _get(self, cmd, timeout):
        with self._session_lock:
            r = self._session.get(self.url, params={"json": _encode(cmd)}, timeout=timeout)
        r.raise_for_status()
        return r.text

    def _send_loop(self):
        while True:
            with self._cond:
                while self._running and not self._pending:
                    self._cond.wait()
                if not self._pending:
                    return  # closed, and everything queued has been sent
                key = next(iter(self._pending))
                cmd = self._pending.pop(key)
            for _ in range(self.STOP_ATTEMPTS if is_axis_stop(cmd) else 1):
                try:
                    body = self._get(cmd, self.timeout)
                except requests.RequestException as e:
                    self.error = f"HTTP request failed ({type(e).__name__})."
                    continue
                self.error = None
                if cmd["T"] == 105:
                    msg = _parse_feedback(body)
                    if msg is not None:
                        self._store_feedback(msg)
                break

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its joint angles. Returns the dict or None."""
        try:
            body = self._get({"T": 105}, timeout)
        except requests.RequestException as e:
            self.error = f"HTTP request failed ({type(e).__name__})."
            return None
        self.error = None
        msg = _parse_feedback(body)
        if msg is not None:
            self._store_feedback(msg)
        return msg

    def poll_feedback(self):
        """Ask for feedback without waiting; the answer arrives via take_feedback()."""
        self.send({"T": 105})

    def cancel_pending(self):
        """Drop commands that haven't been sent yet."""
        with self._cond:
            self._pending.clear()

    def close(self):
        """Send what is still queued (e.g. a final stop), then shut down."""
        with self._cond:
            self._running = False
            self._pending.pop((105, None), None)  # no one will read the answer
            self._cond.notify()
        self._worker.join(timeout=self.STOP_ATTEMPTS * self.timeout + 0.5)
        self._session.close()


# ---------------------------------------------------------------------------
# Recordings
# ---------------------------------------------------------------------------
# A recording is every command sent while recording, with its time in seconds
# from the start of the recording, plus the joint angles when it started.

def save_recording(path, data):
    """Write a recording as JSON, one command per line so the file stays readable."""
    head = {k: v for k, v in data.items() if k != "commands"}
    lines = [json.dumps(c, separators=(",", ":")) for c in data["commands"]]
    text = (json.dumps(head, indent=2)[:-2]
            + ',\n  "commands": [\n    ' + ",\n    ".join(lines) + "\n  ]\n}\n")
    with open(path, "w") as f:
        f.write(text)


def _check_command(cmd):
    """Return a clean copy of a recorded command, or raise ValueError."""
    kind = cmd["T"]
    if kind == 101:
        c = JOINTS_BY_NUMBER[int(cmd["joint"])]
        out = c.command(clamp(c, float(cmd["rad"])))  # never leave the joint range
        out["spd"], out["acc"] = int(cmd.get("spd", CMD_SPD)), int(cmd.get("acc", CMD_ACC))
        return out
    if kind == 123:
        if int(cmd.get("m", 1)) != 1:
            raise ValueError("only T:123 commands with m=1 (coordinate control) are supported")
        c = AXES_BY_NUMBER[int(cmd["axis"])]
        direction = AXIS_DIRECTION_OF[int(cmd["cmd"])]
        spd = max(0.0, float(cmd.get("spd", AXIS_CMD_SPD)))
        return c.axis_command(direction, int(spd) if spd.is_integer() else spd)
    if kind == 103:
        # Older recordings used T:103 position commands; they still play.
        c = AXES_BY_NUMBER[int(cmd["axis"])]
        pos = clamp(c, float(cmd["pos"]))  # never leave the axis range
        return {"T": 103, "axis": c.number, "pos": round(pos, 2 if c.linear else 4),
                "spd": float(cmd.get("spd", 0.25))}
    if kind == 100:
        return {"T": 100}
    if kind == 114:
        return {"T": 114, "led": int(cmd["led"])}
    raise ValueError(f"unsupported command T:{kind}")


def load_recording(path):
    """Read and check a recording file. Raises OSError or ValueError."""
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, dict) or data.get("format") != RECORDING_FORMAT:
        raise ValueError("not a RoArm-M3 recording")
    if data.get("version") != RECORDING_VERSION:
        raise ValueError(f"unsupported recording version {data.get('version')!r}")
    try:
        start = {j: clamp(j, float(data["start"][str(j.number)])) for j in JOINTS}
        commands = [{"t": max(0.0, float(item["t"])), "cmd": _check_command(item["cmd"])}
                    for item in data["commands"]]
        duration = float(data["duration"])
        led = bool(data.get("led", False))
    except (KeyError, TypeError, AttributeError) as e:
        raise ValueError(f"missing or invalid field ({e})")
    if not commands:
        raise ValueError("the recording is empty")
    commands.sort(key=lambda c: c["t"])
    return {
        "start": start,
        "led": led,
        "uses_led": any(c["cmd"]["T"] == 114 for c in commands),
        "duration": max(duration, commands[-1]["t"]),
        "commands": commands,
    }


class DialogTask:
    """Runs a file dialog in a background thread so the window keeps responding."""

    def __init__(self, fn, on_done, **kwargs):
        self.on_done = on_done  # called as on_done(path or None, error or None)
        self.result = None
        self.error = None
        self.done = threading.Event()
        threading.Thread(target=self._run, args=(fn, kwargs), daemon=True).start()

    def _run(self, fn, kwargs):
        try:
            self.result = fn(**kwargs) or None  # the dialog returns "" when cancelled
        except Exception as e:
            self.error = e
        self.done.set()


@dataclass
class Playback:
    rec: dict
    name: str
    loops: int                # 0 = forever
    phase_t0: float
    loop: int = 1
    phase: str = "to_start"   # to_start -> settle -> play
    synced: bool = False      # joint angles read from the arm for this loop
    index: int = 0            # next command to play


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
NO_FEEDBACK = "No feedback from arm, so its position is unknown. Try again, or press P."


class KeyboardController:
    def __init__(self, link):
        self.link = link
        self.mode = "joint"
        every = JOINTS + AXES
        self.target = {c: c.home for c in every}
        # Treat the initial targets as already sent so nothing moves until a key is pressed.
        self.last_sent = dict(self.target)
        self.last_send_time = {c: 0.0 for c in every}
        self.pressed_at = {}  # key -> (time it went down, control, direction)
        self.key_map = {mode: {} for mode in CONTROLS}
        for mode, controls in CONTROLS.items():
            for c in controls:
                self.key_map[mode][c.key_minus] = (c, -1)
                self.key_map[mode][c.key_plus] = (c, +1)
        # Modes whose targets no longer match the arm, because the other mode
        # moved it. They are re-read from the arm before they are used.
        self.stale = set() if link.simulated else {"axis"}
        # Axis mode (T:123): the direction each axis was last told to move
        # (0 = stopped), and when a tap's short pulse of motion should stop.
        self.axis_dir = {a: 0 for a in AXES}
        self.axis_stop_at = {}
        self.next_poll = 0.0             # earliest time for the next position read
        self.poll_after_stop = False     # read the final position once axes stop
        # Mouse sliders: the value each slider-driven control is heading for,
        # and the control whose slider is being dragged (or None).
        self.slider_goal = {}
        self.dragging = None
        self.last_feedback = 0.0         # when a position was last read back
        self.led_on = False  # the arm can't report its LED state; main() turns it off at startup
        self.status = ""
        self.recording = None  # dict while recording
        self.dialog = None     # DialogTask while a file dialog is open
        self.prompt = None     # dict while asking for the number of loops
        self.playback = None   # Playback while playing

    @property
    def controls(self):
        return CONTROLS[self.mode]

    def _send(self, cmd):
        self.link.send(cmd)
        if not self.link.simulated:
            if cmd["T"] in (100, 101):
                self.stale.add("axis")
            elif cmd["T"] in (103, 123):
                self.stale.add("joint")
        if self.recording is not None:
            t = round(time.monotonic() - self.recording["t0"], 3)
            self.recording["commands"].append({"t": t, "cmd": cmd})

    # -- state ------------------------------------------------------------
    def _apply_feedback(self, fb):
        for c in JOINTS + AXES:
            if c.feedback_key in fb:
                self.target[c] = self.last_sent[c] = float(fb[c.feedback_key])
        self.stale.clear()

    def sync_from_arm(self):
        fb = self.link.request_feedback()
        if fb is None:
            self.status = "No feedback from arm; using the last known positions."
            return False
        self._apply_feedback(fb)
        self.status = "Synced positions from arm feedback."
        return True

    def ensure_synced(self, mode):
        """Make sure the targets for mode match the arm. False if the arm doesn't answer."""
        if mode not in self.stale:
            return True
        fb = self.link.request_feedback()
        if fb is None:
            return False
        self._apply_feedback(fb)
        return True

    def hold_position(self):
        """Stop the arm where it is now by commanding each joint to its measured angle."""
        fb = self.link.request_feedback(timeout=0.5)
        if fb is None:
            return False
        now = time.monotonic()
        for j in JOINTS:
            if j.feedback_key in fb:
                self.target[j] = float(fb[j.feedback_key])
                self._send_control(j, now, force=True)
        for a in AXES:
            if a.feedback_key in fb:
                self.target[a] = self.last_sent[a] = float(fb[a.feedback_key])
        self.stale.clear()
        return True

    def go_home(self):
        self.pressed_at.clear()
        self.slider_goal.clear()
        self.dragging = None
        self.stop_all_axes(time.monotonic())
        self._send({"T": 100})
        for c in JOINTS + AXES:
            self.target[c] = self.last_sent[c] = c.home
        self.status = "Moving to home position."

    def toggle_led(self):
        self.set_led(not self.led_on)
        self.status = "LED on." if self.led_on else "LED off."

    def set_led(self, on):
        self.led_on = on
        self._send({"T": 114, "led": LED_ON if on else LED_OFF})

    def toggle_mode(self, now):
        new = "axis" if self.mode == "joint" else "joint"
        self.release_all_keys(now)
        if not self.ensure_synced(new):
            self.status = f"Can't switch modes. {NO_FEEDBACK}"
            return
        self.mode = new
        self.status = f"{MODE_LABEL[new]}."

    # -- input ------------------------------------------------------------
    def handle_key(self, key, char, now):
        """Handle a key press. Returns False when the program should quit."""
        if self.dialog:
            return True  # waiting for the file dialog
        if self.prompt:
            self._prompt_key(key, char, now)
            return True
        if self.playback:
            if key in STOP_KEYS:
                self.stop_playback()
            return True
        if key == QUIT_KEY:
            return False
        if key == HOME_KEY:
            self.go_home()
        elif key == SYNC_KEY:
            self.sync_from_arm()
        elif key == LED_KEY:
            self.toggle_led()
        elif key == MODE_KEY:
            self.toggle_mode(now)
        elif key == RECORD_KEY:
            self.toggle_recording(now)
        elif key == PLAY_KEY:
            self.choose_recording(now)
        else:
            self.key_down(key, now)
        return True

    def key_down(self, key, now):
        entry = self.key_map[self.mode].get(key)
        if entry is None or key in self.pressed_at:
            return
        if not self.ensure_synced(self.mode):
            self.status = NO_FEEDBACK
            return
        control, direction = entry
        self.pressed_at[key] = (now, control, direction)
        # A key takes over from the mouse slider for this control.
        self.slider_goal.pop(control, None)
        if self.dragging is control:
            self.dragging = None
        if control.mode == "axis":
            # T:123 keeps the axis moving until it is told to stop.
            self.axis_stop_at.pop(control, None)
            self._set_axis(control, direction, now)
            return
        # Every press gives one small step right away, so a tap always moves.
        self._move(control, direction * TAP_STEP)
        self._send_control(control, now, force=True)

    def key_up(self, key, now):
        if key not in self.pressed_at:
            return
        t0, control, _ = self.pressed_at.pop(key)
        if control.mode == "joint":
            # Send the final position so it stops exactly at the target.
            self._send_control(control, now, force=True)
            return
        held = self._held_direction(control)
        if held:
            # The opposite key for this axis is still down: move that way instead.
            self._set_axis(control, held, now)
        elif self.axis_dir[control]:
            # A quick tap still moves for AXIS_TAP_TIME.
            stop_at = t0 + AXIS_TAP_TIME
            if now >= stop_at:
                self._set_axis(control, 0, now)
            else:
                self.axis_stop_at[control] = stop_at

    def release_all_keys(self, now):
        """Let go of every key and slider and stop all motion right away."""
        stopping = [c for _, c, _ in self.pressed_at.values()] + list(self.slider_goal)
        self.pressed_at.clear()
        self.slider_goal.clear()
        self.dragging = None
        for control in stopping:
            if control.mode == "joint":
                self._send_control(control, now, force=True)
        self.stop_all_axes(now)

    # -- mouse sliders ----------------------------------------------------
    def slider_press(self, control, value, now):
        """The mouse button went down on a control's slider at value."""
        if self.dialog or self.prompt or self.playback or control.mode != self.mode:
            return
        if any(c is control for _, c, _ in self.pressed_at.values()):
            return  # a key is already moving this control
        if not self.ensure_synced(self.mode):
            self.status = NO_FEEDBACK
            return
        self.dragging = control
        self._set_slider_goal(control, value, now)

    def slider_drag(self, value, now):
        """The mouse moved while dragging a slider."""
        if self.dragging is not None:
            self._set_slider_goal(self.dragging, value, now)

    def slider_release(self, now):
        """The mouse button came up. The control keeps going to the slider value."""
        self.dragging = None

    def _set_slider_goal(self, control, value, now):
        self.slider_goal[control] = clamp(control, value)
        if control.mode == "axis":
            self._steer_axis(control, now)

    @staticmethod
    def _slider_tolerance(control):
        return SLIDER_TOLERANCE_MM if control.linear else SLIDER_TOLERANCE_RAD

    def _steer_axis(self, control, now, from_feedback=False):
        """Run a slider-driven axis toward its goal, or stop it once it is there.

        from_feedback: called because a new position was read. If the axis has
        passed the goal it stops instead of turning back, so it can't oscillate.
        When the user moves the slider behind the axis, it does turn back."""
        goal = self.slider_goal.get(control)
        if goal is None:
            return
        diff = goal - self.target[control]
        direction = self.axis_dir[control]
        passed = from_feedback and direction and diff * direction < 0
        if abs(diff) <= self._slider_tolerance(control) or passed:
            self._set_axis(control, 0, now)
            return
        if not any(self.axis_dir.values()):
            self.last_feedback = now  # start the no-feedback timer now
        self._set_axis(control, 1 if diff > 0 else -1, now)

    def _update_joint_sliders(self, now, dt):
        """Glide slider-driven joints toward their goals at up to MAX_SPEED."""
        step = MAX_SPEED * dt
        for c, goal in list(self.slider_goal.items()):
            if c.mode != "joint":
                continue
            cur = self.target[c]
            if abs(goal - cur) <= step:
                self.target[c] = goal
                self._send_control(c, now, force=True)
                if c is not self.dragging:
                    del self.slider_goal[c]
            else:
                self.target[c] = cur + math.copysign(step, goal - cur)

    def _move(self, control, delta):
        self.target[control] = step_within_range(control, self.target[control], delta)

    # -- axis mode (T:123) --------------------------------------------------
    def _held_direction(self, control):
        """Direction of the most recently pressed key still held for control, or 0."""
        held = [(t0, d) for t0, c, d in self.pressed_at.values() if c is control]
        return max(held)[1] if held else 0

    @staticmethod
    def _at_limit(control, value, direction):
        return value >= control.maximum if direction > 0 else value <= control.minimum

    def _set_axis(self, control, direction, now, spd=AXIS_CMD_SPD):
        """Start moving an axis (+1 / -1) or stop it (0) with T:123."""
        if direction and self._at_limit(control, self.target[control], direction):
            self.status = f"{control.name} is at the end of its range."
            direction = 0
        if direction == 0:
            self.slider_goal.pop(control, None)
        if direction == self.axis_dir[control]:
            return
        self._send(control.axis_command(direction, spd))
        self.axis_dir[control] = direction
        if direction == 0:
            self.axis_stop_at.pop(control, None)
            # Read the final position once the arm has had a moment to stop.
            self.poll_after_stop = True
            self.next_poll = max(self.next_poll, now + AXIS_POLL_AFTER_STOP)

    def stop_all_axes(self, now):
        self.axis_stop_at.clear()
        for a in AXES:
            if self.axis_dir[a]:
                self._set_axis(a, 0, now)

    def _update_axes(self, now):
        """End tap pulses, read positions while axes move and stop them at their limits."""
        for a, stop_at in list(self.axis_stop_at.items()):
            if now >= stop_at:
                self._set_axis(a, 0, now)
        fb = self.link.take_feedback()
        if fb is not None:
            self.last_feedback = now
            for a in AXES:
                if a.feedback_key in fb:
                    self.target[a] = float(fb[a.feedback_key])
                if self.axis_dir[a] and self._at_limit(a, self.target[a], self.axis_dir[a]):
                    self._set_axis(a, 0, now)
                    self.status = f"Stopped {a.name}: it reached the end of its range."
                elif a in self.slider_goal:
                    self._steer_axis(a, now, from_feedback=True)
        axis_goals = [a for a in self.slider_goal if a.mode == "axis"]
        if axis_goals and self.link.simulated:
            # --dry-run: no positions come back, so jump to the goal once the mouse is released.
            for a in axis_goals:
                if a is not self.dragging:
                    self.target[a] = self.slider_goal[a]
                    self._set_axis(a, 0, now)
        elif any(self.axis_dir[a] for a in axis_goals) and now - self.last_feedback > SLIDER_FEEDBACK_TIMEOUT:
            for a in axis_goals:
                self._set_axis(a, 0, now)
            self.status = "Stopped the slider move: no position feedback from the arm."
        moving = any(self.axis_dir.values())
        if (moving or self.poll_after_stop) and now >= self.next_poll:
            self.link.poll_feedback()
            self.next_poll = now + 1.0 / AXIS_POLL_HZ
            if not moving:
                self.poll_after_stop = False

    # -- recording --------------------------------------------------------
    def toggle_recording(self, now):
        if self.recording is None:
            if any(self.axis_dir.values()):
                # A recording must not begin with an axis already moving, or it
                # would play back without the command that started it.
                self.release_all_keys(now)
            # The start position is stored as joint angles, so they must be current.
            if not self.ensure_synced("joint"):
                self.status = f"Can't start recording. {NO_FEEDBACK}"
                return
            start = {j: self.target[j] for j in JOINTS}
            self.recording = {"t0": now, "start": start, "led": self.led_on, "commands": []}
            return
        self.release_all_keys(now)  # so the final positions are recorded
        rec, self.recording = self.recording, None
        if not rec["commands"]:
            self.status = "Recording stopped: nothing was recorded."
            return
        data = {
            "format": RECORDING_FORMAT,
            "version": RECORDING_VERSION,
            "duration": round(now - rec["t0"], 3),
            "led": rec["led"],
            "start": {str(j.number): round(rad, 4) for j, rad in rec["start"].items()},
            "commands": rec["commands"],
        }
        if crossfiledialog is None:
            self._save_recording(None, RuntimeError("no file dialog available"), data)
            return
        self.status = "Choose where to save the recording in the file dialog."
        self.dialog = DialogTask(crossfiledialog.save_file,
                                 lambda path, error: self._save_recording(path, error, data),
                                 title="Save recording", start_dir=os.getcwd())

    def _save_recording(self, path, error, data):
        if error is not None:
            # Don't lose the recording: save it in the current folder instead.
            path = time.strftime("recording-%Y%m%d-%H%M%S.json")
        elif not path:
            self.status = "Save cancelled; the recording was discarded."
            return
        if not os.path.splitext(path)[1]:
            path += ".json"
        try:
            save_recording(path, data)
        except OSError as e:
            self.status = f"Could not save the recording: {e.strerror}."
            return
        self.status = (f"Saved {os.path.basename(path)} "
                       f"({data['duration']:.1f} s, {len(data['commands'])} commands).")
        if error is not None:
            self.status = f"File dialog failed; {self.status[0].lower()}{self.status[1:]}"

    # -- playback ---------------------------------------------------------
    def choose_recording(self, now):
        if self.recording is not None:
            self.status = "Stop recording (M) before playing a recording."
            return
        if crossfiledialog is None:
            self.status = "No file dialog available. Install zenity or kdialog."
            return
        self.release_all_keys(now)
        self.status = "Choose a recording to play in the file dialog."
        self.dialog = DialogTask(crossfiledialog.open_file, self._recording_chosen,
                                 title="Play recording", start_dir=os.getcwd(),
                                 filter={"Recordings (*.json)": "*.json", "All files": "*"})

    def _recording_chosen(self, path, error):
        if error is not None:
            self.status = f"File dialog failed: {error}"
            return
        if not path:
            self.status = "Playback cancelled."
            return
        name = os.path.basename(path)
        try:
            rec = load_recording(path)
        except (OSError, ValueError) as e:
            self.status = f"Can't play {name}: {e}"
            return
        self.prompt = {"rec": rec, "name": name, "text": ""}

    def _prompt_key(self, key, char, now):
        p = self.prompt
        if key == pygame.K_ESCAPE:
            self.prompt = None
            self.status = "Playback cancelled."
        elif key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            self.prompt = None
            self.start_playback(p["rec"], p["name"], int(p["text"] or "1"), now)
        elif key == pygame.K_BACKSPACE:
            p["text"] = p["text"][:-1]
        elif len(char) == 1 and char in string.digits and len(p["text"]) < 6:
            p["text"] += char

    def start_playback(self, rec, name, loops, now):
        self.release_all_keys(now)
        self.playback = Playback(rec, name, loops, phase_t0=now)

    def stop_playback(self):
        """Interrupt playback and stop the arm where it is."""
        self.playback = None
        self.link.cancel_pending()
        self.stop_all_axes(time.monotonic())
        self.hold_position()
        self.status = "Playback stopped."

    def _update_playback(self, now, dt):
        pb = self.playback
        rec = pb.rec
        if pb.phase == "to_start":
            if not pb.synced:
                # The glide starts from the current joint angles, so they must be
                # re-read if axis commands (T:123) moved the arm.
                if not self.ensure_synced("joint"):
                    self.playback = None
                    self.status = f"Playback stopped. {NO_FEEDBACK}"
                    return
                pb.synced = True
            # Glide every joint to where the recording starts.
            step = PLAYBACK_MOVE_SPEED * dt
            arrived = True
            for j in JOINTS:
                goal, cur = rec["start"][j], self.target[j]
                if abs(goal - cur) <= step:
                    self.target[j] = goal
                else:
                    self.target[j] = cur + math.copysign(step, goal - cur)
                    arrived = False
                self._send_control(j, now, force=arrived)
            if arrived:
                if rec["uses_led"] and self.led_on != rec["led"]:
                    self.set_led(rec["led"])
                pb.phase, pb.phase_t0 = "settle", now
        elif pb.phase == "settle":
            if now - pb.phase_t0 >= PLAYBACK_SETTLE:
                pb.phase, pb.phase_t0, pb.index = "play", now, 0
        else:
            elapsed = now - pb.phase_t0
            commands = rec["commands"]
            while pb.index < len(commands) and commands[pb.index]["t"] <= elapsed:
                self._play_command(commands[pb.index]["cmd"], now)
                pb.index += 1
            if pb.index >= len(commands) and elapsed >= rec["duration"]:
                # In case the recording left an axis moving (e.g. an edited file).
                self.stop_all_axes(now)
                if pb.loops and pb.loop >= pb.loops:
                    self.playback = None
                    self.status = f"Finished playing {pb.name}."
                else:
                    pb.loop += 1
                    pb.phase, pb.phase_t0, pb.synced = "to_start", now, False

    def _play_command(self, cmd, now):
        kind = cmd["T"]
        if kind in (101, 103):
            c = JOINTS_BY_NUMBER[cmd["joint"]] if kind == 101 else AXES_BY_NUMBER[cmd["axis"]]
            value = cmd["rad"] if kind == 101 else cmd["pos"]
            self.target[c] = self.last_sent[c] = value
            self.last_send_time[c] = now
            self._send(cmd)
        elif kind == 123:
            # Goes through _set_axis so the range limits still apply.
            self._set_axis(AXES_BY_NUMBER[cmd["axis"]], AXIS_DIRECTION_OF[cmd["cmd"]], now, cmd["spd"])
        elif kind == 100:
            self.go_home()
        elif kind == 114:
            self.set_led(cmd["led"] != LED_OFF)

    # -- loop -------------------------------------------------------------
    def update(self, now, dt):
        if self.dialog and self.dialog.done.is_set():
            task, self.dialog = self.dialog, None
            task.on_done(task.result, task.error)
        self._update_axes(now)
        if self.playback:
            self._update_playback(now, dt)
            return
        self._update_joint_sliders(now, dt)
        for t0, control, direction in self.pressed_at.values():
            if control.mode == "joint":
                speed = self.hold_speed(now - t0)
                self._move(control, direction * speed * dt)
        for c in JOINTS:
            self._send_control(c, now)

    @staticmethod
    def hold_speed(held):
        """Joint speed (rad/s) after a key has been held for `held` seconds."""
        if held < HOLD_DELAY:
            return 0.0
        return min(MAX_SPEED, START_SPEED + ACCELERATION * (held - HOLD_DELAY))

    def _send_control(self, control, now, force=False):
        """Send a joint's target with T:101 if it changed (at most SEND_HZ per second)."""
        value = self.target[control]
        if abs(value - self.last_sent[control]) < 1e-4:
            return
        if not force and now - self.last_send_time[control] < 1.0 / SEND_HZ:
            return
        self._send(control.command(value))
        self.last_sent[control] = value
        self.last_send_time[control] = now

    def joint_speed(self, control, now):
        """Current commanded speed of a joint in rad/s (for display)."""
        return sum(direction * self.hold_speed(now - t0)
                   for t0, c, direction in self.pressed_at.values() if c is control)

    def status_lines(self, now):
        """Lines of (text, color) for the bottom of the window."""
        def short(name):
            return name if len(name) <= 40 else name[:37] + "..."

        if self.prompt:
            rec = self.prompt["rec"]
            return [
                (f"Play {short(self.prompt['name'])} ({rec['duration']:.1f} s)", COLOR_PROMPT),
                (f"Loops (Enter = once, 0 = forever, Esc = cancel): {self.prompt['text']}_", COLOR_PROMPT),
            ]
        if self.playback:
            pb = self.playback
            total = "forever" if pb.loops == 0 else f"of {pb.loops}"
            if pb.phase == "play":
                where = f"{min(now - pb.phase_t0, pb.rec['duration']):.1f} / {pb.rec['duration']:.1f} s"
            else:
                where = "moving to start position"
            return [
                (f"Playing {short(pb.name)}, loop {pb.loop} {total}", COLOR_PLAY),
                (f"{where}   Space/Esc = stop", COLOR_PLAY),
            ]
        if self.recording is not None:
            rec = self.recording
            return [
                (f"REC  {now - rec['t0']:.1f} s, {len(rec['commands'])} commands", COLOR_REC),
                ("M = stop and save", COLOR_REC),
            ]
        return [(self.status, COLOR_STATUS)] if self.status else []


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------
COLOR_STATUS = (240, 200, 120)
COLOR_PROMPT = (255, 255, 255)
COLOR_PLAY = (120, 220, 140)
COLOR_REC = (255, 90, 90)
COLOR_ERROR = (255, 110, 110)
COLOR_MODE = {"joint": (200, 220, 255), "axis": (255, 205, 150)}
COLOR_BAR = {"joint": (90, 170, 255), "axis": (255, 160, 70)}
COLOR_GOAL = (235, 235, 235)

# Layout of the control rows. Each row is ROW_H tall: a line of text, then
# the slider bar. The mouse grabs a slider anywhere in SLIDER_HIT_TOP..BOTTOM.
ROWS_TOP = 74
ROW_H = 48
BAR_X, BAR_W = 16, 560
BAR_Y = 26                          # bar offset from the top of its row
SLIDER_HIT_TOP, SLIDER_HIT_BOTTOM = 18, 42
SLIDER_HIT_MARGIN = 8               # extra px left/right of the bar


def slider_x(control, value):
    frac = min(1.0, max(0.0, (value - control.minimum) / (control.maximum - control.minimum)))
    return BAR_X + int(frac * BAR_W)


def slider_value(control, x):
    """Value of control at window x position x (clamped to its range)."""
    frac = min(1.0, max(0.0, (x - BAR_X) / BAR_W))
    return control.minimum + frac * (control.maximum - control.minimum)


def slider_at(controls, pos):
    """(control, value) for a mouse position on a slider, or None."""
    x, y = pos
    if not BAR_X - SLIDER_HIT_MARGIN <= x <= BAR_X + BAR_W + SLIDER_HIT_MARGIN:
        return None
    row, offset = divmod(y - ROWS_TOP, ROW_H)
    if 0 <= row < len(controls) and SLIDER_HIT_TOP <= offset <= SLIDER_HIT_BOTTOM:
        return controls[row], slider_value(controls[row], x)
    return None


def draw(screen, font, small, ctrl, now, port_label):
    screen.fill((24, 26, 30))
    y = 16
    screen.blit(font.render(f"RoArm-M3 keyboard control  [{port_label}]", True, (230, 230, 230)), (16, y))
    y += 28
    other = "axis" if ctrl.mode == "joint" else "joint"
    screen.blit(small.render(f"{MODE_LABEL[ctrl.mode]}   Tab = switch to {other} mode",
                             True, COLOR_MODE[ctrl.mode]), (16, y))
    y = ROWS_TOP
    for c in ctrl.controls:
        value = ctrl.target[c]
        minus = pygame.key.name(c.key_minus).upper()
        plus = pygame.key.name(c.key_plus).upper()
        if c.linear:
            text = f"{c.number} {c.name:<9} {minus}/{plus}  {value:+7.1f} mm"
        else:
            text = f"{c.number} {c.name:<9} {minus}/{plus}  {value:+.3f} rad  {math.degrees(value):+7.1f} deg"
        if c.mode == "joint":
            speed = ctrl.joint_speed(c, now)
            if speed:
                text += f"   {math.degrees(speed):+.0f} deg/s"
        elif ctrl.axis_dir[c]:
            text += "   moving " + ("+" if ctrl.axis_dir[c] > 0 else "-")
        screen.blit(font.render(text, True, COLOR_MODE[ctrl.mode]), (16, y))
        # range bar (a slider: click or drag it with the mouse)
        pygame.draw.rect(screen, (60, 64, 72), (BAR_X, y + BAR_Y, BAR_W, 6))
        goal = ctrl.slider_goal.get(c)
        if goal is not None:
            # Where the slider is taking this control.
            pygame.draw.rect(screen, COLOR_GOAL, (slider_x(c, goal) - 5, y + BAR_Y - 6, 10, 18), 2)
        pygame.draw.rect(screen, COLOR_BAR[ctrl.mode], (slider_x(c, value) - 3, y + BAR_Y - 4, 6, 14))
        y += ROW_H
    help_lines = [
        "Tap a key for a small step; hold it to move faster and faster."
        if ctrl.mode == "joint" else "Hold a key to move; release it to stop.",
        "Or click / drag a slider with the mouse.",
        "0 = home (T:100)   P = re-sync   L = LED   Esc = quit",
        "M = record on/off   B = play a recording",
    ]
    y += 4
    for line in help_lines:
        screen.blit(small.render(line, True, (160, 160, 160)), (16, y))
        y += 22
    y += 4
    for text, color in ctrl.status_lines(now):
        screen.blit(small.render(text, True, color), (16, y))
        y += 22
    if ctrl.link.error:
        screen.blit(small.render(ctrl.link.error, True, COLOR_ERROR), (16, y))
    pygame.display.flip()


def main():
    parser = argparse.ArgumentParser(description="Keyboard controller for the Waveshare RoArm-M3")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    parser.add_argument("--http", nargs="?", const=DEFAULT_HTTP_HOST, metavar="IP",
                        help=f"use HTTP over WiFi instead of serial; IP defaults to {DEFAULT_HTTP_HOST}")
    parser.add_argument("--dry-run", action="store_true", help="don't connect; print commands instead")
    args = parser.parse_args()

    if args.dry_run:
        link = DryRunLink()
        port_label = "dry run"
    elif args.http:
        link = HttpLink(args.http)
        port_label = f"http://{args.http}"
    else:
        link = SerialLink(args.port, args.baud)
        port_label = args.port
        time.sleep(0.5)  # let the board settle after opening the port

    ctrl = KeyboardController(link)
    # The arm can't report its LED state, so start from a known one: off.
    ctrl.set_led(False)
    if args.dry_run:
        ctrl.status = "Dry run: commands are printed to the terminal."
    else:
        ctrl.sync_from_arm()

    pygame.init()
    screen = pygame.display.set_mode((600, 545))
    pygame.display.set_caption("RoArm-M3 Keyboard Controller")
    font = pygame.font.SysFont("monospace", 18)
    small = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()

    last = time.monotonic()
    running = True
    try:
        while running:
            now = time.monotonic()
            dt = min(now - last, 0.1)  # don't jump after a stall
            last = now
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = ctrl.handle_key(event.key, getattr(event, "unicode", ""), now)
                elif event.type == pygame.KEYUP:
                    ctrl.key_up(event.key, now)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    hit = slider_at(ctrl.controls, event.pos)
                    if hit is not None:
                        ctrl.slider_press(*hit, now)
                elif event.type == pygame.MOUSEMOTION and ctrl.dragging is not None:
                    ctrl.slider_drag(slider_value(ctrl.dragging, event.pos[0]), now)
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    ctrl.slider_release(now)
                elif event.type == pygame.WINDOWFOCUSLOST:
                    # Key-up events are missed when focus is lost; stop everything.
                    ctrl.release_all_keys(now)
            ctrl.update(now, dt)
            draw(screen, font, small, ctrl, now, port_label)
            clock.tick(LOOP_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        # T:123 keeps an axis moving until it is told to stop.
        ctrl.stop_all_axes(time.monotonic())
        pygame.quit()
        link.close()


if __name__ == "__main__":
    main()
