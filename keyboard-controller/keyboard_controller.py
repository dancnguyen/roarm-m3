#!/usr/bin/env python3
"""Keyboard controller for the Waveshare RoArm-M3 over serial.

Each joint is driven with the {"T":101} single-joint command (radians).

- Tap a key: the joint moves one small step.
- Hold a key: after a short delay the joint moves continuously, speeding up
  the longer the key is held (up to a maximum speed).

Keys are captured from a small pygame window (works on both X11 and Wayland),
so the window must have focus.

Usage:
    python keyboard_controller.py --port /dev/ttyUSB0
    python keyboard_controller.py --dry-run      # no arm, print commands
"""

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass

import pygame

try:
    import serial
except ImportError:  # only needed when not in dry-run mode
    serial = None


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
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


@dataclass
class Joint:
    number: int      # joint id used by T:101
    name: str
    minimum: float   # rad
    maximum: float   # rad
    home: float      # rad
    key_minus: int
    key_plus: int
    feedback_key: str  # field name in the T:1051 feedback message


# Ranges and home positions are from the Waveshare RoArm-M3 JSON command docs.
JOINTS = [
    Joint(1, "Base",     -3.14,  3.14, 0.0,      pygame.K_a, pygame.K_q, "b"),
    Joint(2, "Shoulder", -1.57,  1.57, 0.0,      pygame.K_s, pygame.K_w, "s"),
    Joint(3, "Elbow",     0.0,   3.14, math.pi / 2, pygame.K_d, pygame.K_e, "e"),
    Joint(4, "Wrist",    -1.57,  1.57, 0.0,      pygame.K_f, pygame.K_r, "t"),
    Joint(5, "Roll",     -3.14,  3.14, 0.0,      pygame.K_g, pygame.K_t, "r"),
    Joint(6, "Gripper",   1.08,  3.14, math.pi,  pygame.K_h, pygame.K_y, "g"),
]

HOME_KEY = pygame.K_0
SYNC_KEY = pygame.K_p
QUIT_KEY = pygame.K_ESCAPE


# ---------------------------------------------------------------------------
# Serial link
# ---------------------------------------------------------------------------
class ArmLink:
    """Sends JSON commands and reads JSON feedback in a background thread."""

    def __init__(self, port, baud, dry_run=False):
        self.dry_run = dry_run
        self.feedback = None          # latest T:1051 message (dict)
        self.feedback_event = threading.Event()
        self._lock = threading.Lock()
        self._running = True
        self._ser = None
        if dry_run:
            return
        if serial is None:
            sys.exit("pyserial is not installed. Run: pip install -r requirements.txt")
        self._ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=None)
        # Avoid resetting the ESP32 when the port opens.
        self._ser.setRTS(False)
        self._ser.setDTR(False)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def send(self, cmd: dict):
        line = json.dumps(cmd, separators=(",", ":"))
        if self.dry_run:
            print("TX", line)
            return
        with self._lock:
            self._ser.write((line + "\n").encode())

    def _read_loop(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                msg = json.loads(raw.decode(errors="ignore").strip())
            except ValueError:
                continue
            if isinstance(msg, dict) and msg.get("T") == 1051:
                self.feedback = msg
                self.feedback_event.set()

    def request_feedback(self, timeout=1.0):
        """Ask the arm for its joint angles. Returns the dict or None."""
        if self.dry_run:
            return None
        self.feedback_event.clear()
        self.send({"T": 105})
        if self.feedback_event.wait(timeout):
            return self.feedback
        return None

    def close(self):
        self._running = False
        if self._ser is not None:
            self._ser.close()


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
class KeyboardController:
    def __init__(self, link: ArmLink):
        self.link = link
        self.target = {j.number: j.home for j in JOINTS}
        # Treat the initial targets as already sent so nothing moves until a key is pressed.
        self.last_sent = dict(self.target)
        self.last_send_time = {j.number: 0.0 for j in JOINTS}
        self.pressed_at = {}  # key -> time it went down
        self.key_to_joint = {}
        for j in JOINTS:
            self.key_to_joint[j.key_minus] = (j, -1)
            self.key_to_joint[j.key_plus] = (j, +1)
        self.status = ""

    # -- state ------------------------------------------------------------
    def sync_from_arm(self):
        fb = self.link.request_feedback()
        if fb is None:
            self.status = "No feedback from arm; using home positions as targets."
            return False
        for j in JOINTS:
            if j.feedback_key in fb:
                self.target[j.number] = self._clamp(j, float(fb[j.feedback_key]))
                self.last_sent[j.number] = self.target[j.number]
        self.status = "Synced targets from arm feedback."
        return True

    def go_home(self):
        self.pressed_at.clear()
        self.link.send({"T": 100})
        for j in JOINTS:
            self.target[j.number] = j.home
            self.last_sent[j.number] = j.home
        self.status = "Moving to home position."

    @staticmethod
    def _clamp(joint, value):
        return max(joint.minimum, min(joint.maximum, value))

    def _move(self, joint, delta):
        self.target[joint.number] = self._clamp(joint, self.target[joint.number] + delta)

    # -- input ------------------------------------------------------------
    def key_down(self, key, now):
        if key not in self.key_to_joint or key in self.pressed_at:
            return
        self.pressed_at[key] = now
        joint, direction = self.key_to_joint[key]
        # Every press gives one small step right away, so a tap always moves.
        self._move(joint, direction * TAP_STEP)
        self._send_joint(joint, now, force=True)

    def key_up(self, key, now):
        if key in self.pressed_at:
            del self.pressed_at[key]
            joint, _ = self.key_to_joint[key]
            # Send the final position so the joint stops exactly at the target.
            self._send_joint(joint, now, force=True)

    # -- loop -------------------------------------------------------------
    def update(self, now, dt):
        for key, t0 in self.pressed_at.items():
            held = now - t0
            if held < HOLD_DELAY:
                continue
            joint, direction = self.key_to_joint[key]
            speed = min(MAX_SPEED, START_SPEED + ACCELERATION * (held - HOLD_DELAY))
            self._move(joint, direction * speed * dt)
        for j in JOINTS:
            self._send_joint(j, now)

    def _send_joint(self, joint, now, force=False):
        n = joint.number
        rad = round(self.target[n], 4)
        if self.last_sent[n] is not None and abs(rad - self.last_sent[n]) < 1e-4:
            return
        if not force and now - self.last_send_time[n] < 1.0 / SEND_HZ:
            return
        self.link.send({"T": 101, "joint": n, "rad": rad, "spd": CMD_SPD, "acc": CMD_ACC})
        self.last_sent[n] = rad
        self.last_send_time[n] = now

    def joint_speed(self, joint, now):
        """Current commanded speed of a joint in rad/s (for display)."""
        speed = 0.0
        for key, t0 in self.pressed_at.items():
            j, direction = self.key_to_joint[key]
            if j is joint and now - t0 >= HOLD_DELAY:
                held = now - t0 - HOLD_DELAY
                speed += direction * min(MAX_SPEED, START_SPEED + ACCELERATION * held)
        return speed


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------
def draw(screen, font, small, ctrl, now, port_label):
    screen.fill((24, 26, 30))
    y = 16
    screen.blit(font.render(f"RoArm-M3 keyboard control  [{port_label}]", True, (230, 230, 230)), (16, y))
    y += 40
    for j in JOINTS:
        rad = ctrl.target[j.number]
        minus = pygame.key.name(j.key_minus).upper()
        plus = pygame.key.name(j.key_plus).upper()
        speed = ctrl.joint_speed(j, now)
        text = f"{j.number} {j.name:<9} {minus}/{plus}  {rad:+.3f} rad  {math.degrees(rad):+7.1f} deg"
        if speed:
            text += f"   {math.degrees(speed):+.0f} deg/s"
        screen.blit(font.render(text, True, (200, 220, 255)), (16, y))
        # range bar
        bar_x, bar_w = 16, 560
        frac = (rad - j.minimum) / (j.maximum - j.minimum)
        pygame.draw.rect(screen, (60, 64, 72), (bar_x, y + 26, bar_w, 6))
        pygame.draw.rect(screen, (90, 170, 255), (bar_x + int(frac * bar_w) - 3, y + 22, 6, 14))
        y += 48
    help_lines = [
        "Tap a key for a small step; hold it to move faster and faster.",
        "0 = home (T:100)   P = re-sync from arm   Esc = quit",
    ]
    y += 4
    for line in help_lines:
        screen.blit(small.render(line, True, (160, 160, 160)), (16, y))
        y += 22
    if ctrl.status:
        screen.blit(small.render(ctrl.status, True, (240, 200, 120)), (16, y + 4))
    pygame.display.flip()


def main():
    parser = argparse.ArgumentParser(description="Keyboard controller for the Waveshare RoArm-M3")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    parser.add_argument("--dry-run", action="store_true", help="no serial; print commands instead")
    args = parser.parse_args()

    link = ArmLink(args.port, args.baud, dry_run=args.dry_run)
    ctrl = KeyboardController(link)
    if not args.dry_run:
        time.sleep(0.5)  # let the board settle after opening the port
        ctrl.sync_from_arm()
    else:
        ctrl.status = "Dry run: commands are printed to the terminal."

    pygame.init()
    screen = pygame.display.set_mode((600, 420))
    pygame.display.set_caption("RoArm-M3 Keyboard Controller")
    font = pygame.font.SysFont("monospace", 18)
    small = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()
    port_label = "dry run" if args.dry_run else args.port

    last = time.monotonic()
    running = True
    try:
        while running:
            now = time.monotonic()
            dt = now - last
            last = now
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == QUIT_KEY:
                        running = False
                    elif event.key == HOME_KEY:
                        ctrl.go_home()
                    elif event.key == SYNC_KEY:
                        ctrl.sync_from_arm()
                    else:
                        ctrl.key_down(event.key, now)
                elif event.type == pygame.KEYUP:
                    ctrl.key_up(event.key, now)
                elif event.type == pygame.WINDOWFOCUSLOST:
                    # Key-up events are missed when focus is lost; stop everything.
                    for key in list(ctrl.pressed_at):
                        ctrl.key_up(key, now)
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
