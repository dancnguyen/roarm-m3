#!/usr/bin/env python3
"""Keyboard controller for the Waveshare RoArm-M3 over serial or HTTP.

Two control modes (Tab switches between them):
- Joint mode: each key pair drives one joint with {"T":101} (radians).
- Axis mode: each key pair moves the gripper tip along one axis with
  {"T":103}: X, Y, Z (mm), pitch, roll and gripper (radians).

- Tap a key: the joint/axis moves one small step.
- Hold a key: after a short delay it moves continuously, speeding up
  the longer the key is held (up to a maximum speed).
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
# Angles (joints, and pitch/roll/gripper in axis mode)
TAP_STEP = math.radians(2.0)     # rad moved by a quick tap
HOLD_DELAY = 0.25                # s before a held key starts continuous motion
START_SPEED = math.radians(10)   # rad/s when continuous motion begins
ACCELERATION = math.radians(40)  # rad/s^2 added while the key stays held
MAX_SPEED = math.radians(90)     # rad/s cap

# Distances (X, Y, Z in axis mode)
LINEAR_TAP_STEP = 5.0            # mm moved by a quick tap
LINEAR_START_SPEED = 20.0        # mm/s when continuous motion begins
LINEAR_ACCELERATION = 60.0       # mm/s^2 added while the key stays held
LINEAR_MAX_SPEED = 100.0         # mm/s cap

LOOP_HZ = 50                     # control loop rate
SEND_HZ = 25                     # max T:101 commands per second per joint
# T:103 makes the arm finish each move before it reads the next command, so
# axis mode sends fewer, larger steps while a key is held.
AXIS_SEND_HZ = 5                 # max T:103 commands per second per axis

# Passed through to T:101. spd = steps/s (0 = max), acc = 100 steps/s^2 (0 = max).
# Targets are updated in small increments, so letting the servo run at max
# speed between them gives the smoothest result.
CMD_SPD = 0
CMD_ACC = 0

# Passed through to T:103. Larger is faster; 0.25 is the value in Waveshare's example.
AXIS_CMD_SPD = 0.25

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
    """One key pair: a joint (T:101) or an axis of the gripper tip (T:103)."""
    mode: str          # "joint" or "axis"
    number: int        # joint id for T:101, axis id for T:103
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
        if self.mode == "joint":
            return {"T": 101, "joint": self.number, "rad": round(value, 4), "spd": CMD_SPD, "acc": CMD_ACC}
        return {"T": 103, "axis": self.number, "pos": round(value, 2 if self.linear else 4), "spd": AXIS_CMD_SPD}


# Joint ranges and home positions are from the Waveshare RoArm-M3 JSON command docs.
JOINTS = [
    Control("joint", 1, "Base",     -3.14,  3.14, 0.0,         pygame.K_a, pygame.K_q, "b"),
    Control("joint", 2, "Shoulder", -1.57,  1.57, 0.0,         pygame.K_s, pygame.K_w, "s"),
    Control("joint", 3, "Elbow",     0.0,   3.14, math.pi / 2, pygame.K_d, pygame.K_e, "e"),
    Control("joint", 4, "Wrist",    -1.57,  1.57, 0.0,         pygame.K_f, pygame.K_r, "t"),
    Control("joint", 5, "Roll",     -3.14,  3.14, 0.0,         pygame.K_g, pygame.K_t, "r"),
    Control("joint", 6, "Gripper",   1.08,  3.14, math.pi,     pygame.K_h, pygame.K_y, "g"),
]

# Axes for T:103, using the same keys as the joints. X points straight ahead of
# the arm, Y to its left and Z up. The docs give no X/Y/Z limits, so these are
# cautious guesses: widen them once you know your arm's workspace. The X/Y/Z
# home values come from the feedback example in the docs and are only used
# when the arm can't report its position (e.g. --dry-run).
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
MODE_LABEL = {"joint": "Joint mode (T:101)", "axis": "Axis mode (T:103)"}


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
# Every link has the same interface: send(cmd), request_feedback(),
# cancel_pending(), close(), an `error` attribute holding the last
# communication error (or None), and `simulated` (True only for --dry-run).


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

    def cancel_pending(self):
        pass

    def close(self):
        pass


class SerialLink:
    """Sends JSON commands over USB serial and reads feedback in a background thread."""

    simulated = False

    def __init__(self, port, baud):
        if serial is None:
            sys.exit("pyserial is not installed. Run: pip install -r requirements.txt")
        self.error = None
        self.feedback = None          # latest T:1051 message (dict)
        self.feedback_event = threading.Event()
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
                self.feedback = msg
                self.feedback_event.set()

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its joint angles. Returns the dict or None."""
        self.feedback_event.clear()
        self.send({"T": 105})
        if self.feedback_event.wait(timeout):
            return self.feedback
        return None

    def cancel_pending(self):
        pass  # serial writes are sent immediately

    def close(self):
        self._running = False
        self._ser.close()


class HttpLink:
    """Sends JSON commands as HTTP GET requests to http://<host>/js?json=<cmd>.

    Requests run in a background thread so a slow network never freezes the
    window. If commands are queued faster than the arm answers, an unsent
    command is replaced by a newer one of the same kind (the same joint for
    T:101, the same axis for T:103), so the arm always gets the latest target
    instead of a backlog.
    """

    simulated = False

    def __init__(self, host, timeout=HTTP_TIMEOUT):
        if requests is None:
            sys.exit("requests is not installed. Run: pip install -r requirements.txt")
        self.url = f"http://{host}/js"
        self.timeout = timeout
        self.error = None
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
            # Remove and re-add so the newest command goes to the back of the queue.
            self._pending.pop(key, None)
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
                if not self._running:
                    return
                key = next(iter(self._pending))
                cmd = self._pending.pop(key)
            try:
                self._get(cmd, self.timeout)
                self.error = None
            except requests.RequestException as e:
                self.error = f"HTTP request failed ({type(e).__name__})."

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its joint angles. Returns the dict or None."""
        try:
            body = self._get({"T": 105}, timeout)
        except requests.RequestException as e:
            self.error = f"HTTP request failed ({type(e).__name__})."
            return None
        self.error = None
        return _parse_feedback(body)

    def cancel_pending(self):
        """Drop commands that haven't been sent yet."""
        with self._cond:
            self._pending.clear()

    def close(self):
        with self._cond:
            self._running = False
            self._cond.notify()
        self._worker.join(timeout=self.timeout + 0.5)
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
    if kind == 103:
        c = AXES_BY_NUMBER[int(cmd["axis"])]
        out = c.command(clamp(c, float(cmd["pos"])))  # never leave the axis range
        out["spd"] = float(cmd.get("spd", AXIS_CMD_SPD))
        return out
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
            elif cmd["T"] == 103:
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
        # Every press gives one small step right away, so a tap always moves.
        step = LINEAR_TAP_STEP if control.linear else TAP_STEP
        self._move(control, direction * step)
        self._send_control(control, now, force=True)

    def key_up(self, key, now):
        if key in self.pressed_at:
            _, control, _ = self.pressed_at.pop(key)
            # Send the final position so it stops exactly at the target.
            self._send_control(control, now, force=True)

    def release_all_keys(self, now):
        for key in list(self.pressed_at):
            self.key_up(key, now)

    def _move(self, control, delta):
        self.target[control] = step_within_range(control, self.target[control], delta)

    # -- recording --------------------------------------------------------
    def toggle_recording(self, now):
        if self.recording is None:
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
        self.hold_position()
        self.status = "Playback stopped."

    def _update_playback(self, now, dt):
        pb = self.playback
        rec = pb.rec
        if pb.phase == "to_start":
            if not pb.synced:
                # The glide starts from the current joint angles, so they must be
                # re-read if axis commands (T:103) moved the arm.
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
        elif kind == 100:
            self.go_home()
        elif kind == 114:
            self.set_led(cmd["led"] != LED_OFF)

    # -- loop -------------------------------------------------------------
    def update(self, now, dt):
        if self.dialog and self.dialog.done.is_set():
            task, self.dialog = self.dialog, None
            task.on_done(task.result, task.error)
        if self.playback:
            self._update_playback(now, dt)
            return
        for t0, control, direction in self.pressed_at.values():
            speed = self.hold_speed(control, now - t0)
            self._move(control, direction * speed * dt)
        for c in self.controls:
            self._send_control(c, now)

    @staticmethod
    def hold_speed(control, held):
        """Speed (rad/s or mm/s) after a key has been held for `held` seconds."""
        if held < HOLD_DELAY:
            return 0.0
        held -= HOLD_DELAY
        if control.linear:
            return min(LINEAR_MAX_SPEED, LINEAR_START_SPEED + LINEAR_ACCELERATION * held)
        return min(MAX_SPEED, START_SPEED + ACCELERATION * held)

    def _send_control(self, control, now, force=False):
        value = self.target[control]
        if abs(value - self.last_sent[control]) < 1e-4:
            return
        hz = SEND_HZ if control.mode == "joint" else AXIS_SEND_HZ
        if not force and now - self.last_send_time[control] < 1.0 / hz:
            return
        self._send(control.command(value))
        self.last_sent[control] = value
        self.last_send_time[control] = now

    def control_speed(self, control, now):
        """Current commanded speed of a control (for display)."""
        return sum(direction * self.hold_speed(c, now - t0)
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


def draw(screen, font, small, ctrl, now, port_label):
    screen.fill((24, 26, 30))
    y = 16
    screen.blit(font.render(f"RoArm-M3 keyboard control  [{port_label}]", True, (230, 230, 230)), (16, y))
    y += 28
    other = "axis" if ctrl.mode == "joint" else "joint"
    screen.blit(small.render(f"{MODE_LABEL[ctrl.mode]}   Tab = switch to {other} mode",
                             True, COLOR_MODE[ctrl.mode]), (16, y))
    y += 30
    for c in ctrl.controls:
        value = ctrl.target[c]
        minus = pygame.key.name(c.key_minus).upper()
        plus = pygame.key.name(c.key_plus).upper()
        speed = ctrl.control_speed(c, now)
        if c.linear:
            text = f"{c.number} {c.name:<9} {minus}/{plus}  {value:+7.1f} mm"
            if speed:
                text += f"   {speed:+.0f} mm/s"
        else:
            text = f"{c.number} {c.name:<9} {minus}/{plus}  {value:+.3f} rad  {math.degrees(value):+7.1f} deg"
            if speed:
                text += f"   {math.degrees(speed):+.0f} deg/s"
        screen.blit(font.render(text, True, COLOR_MODE[ctrl.mode]), (16, y))
        # range bar
        bar_x, bar_w = 16, 560
        frac = min(1.0, max(0.0, (value - c.minimum) / (c.maximum - c.minimum)))
        pygame.draw.rect(screen, (60, 64, 72), (bar_x, y + 26, bar_w, 6))
        pygame.draw.rect(screen, COLOR_BAR[ctrl.mode], (bar_x + int(frac * bar_w) - 3, y + 22, 6, 14))
        y += 48
    help_lines = [
        "Tap a key for a small step; hold it to move faster and faster.",
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
    screen = pygame.display.set_mode((600, 520))
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
                elif event.type == pygame.WINDOWFOCUSLOST:
                    # Key-up events are missed when focus is lost; stop everything.
                    ctrl.release_all_keys(now)
            ctrl.update(now, dt)
            draw(screen, font, small, ctrl, now, port_label)
            clock.tick(LOOP_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        link.close()


if __name__ == "__main__":
    main()
