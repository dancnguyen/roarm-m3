# RoArm-M3 Keyboard Controller

Control each joint of a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3) from the keyboard, over a USB serial connection or over WiFi with HTTP requests. Movements can be recorded to a JSON file and played back once, a set number of times, or forever.

There are two control modes ([command reference](https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control)). `Tab` switches between them:

- **Joint mode:** each key pair turns one joint, sent as a `{"T":101}` single-joint command:
  ```json
  {"T":101,"joint":1,"rad":0.5,"spd":0,"acc":0}
  ```
- **Axis mode:** each key pair moves the gripper tip along one axis (X, Y, Z, pitch, roll, gripper), sent as a `{"T":123}` continuous-movement command in coordinate mode (`"m":1`). The arm works out the joint angles itself and keeps moving until it gets a stop (`"cmd":0`):
  ```json
  {"T":123,"m":1,"axis":1,"cmd":1,"spd":5}
  ```
  `cmd` is `1` to increase the coordinate, `2` to decrease it and `0` to stop.

## Setup

Requires Python 3.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` installs `pygame-ce` (keyboard window), `pyserial` (serial mode), `requests` (HTTP mode) and `crossfiledialog` (save/open dialogs for recordings).

On Linux, `crossfiledialog` shows the dialogs with `zenity` (GNOME, preinstalled on Ubuntu) or `kdialog` (KDE). If neither is installed, add one, e.g. `sudo apt install zenity`.

### Serial (USB)

The arm usually shows up as `/dev/ttyUSB0` (or `/dev/ttyACM0`). Your user needs access to the serial port. On most Linux distributions:

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

### HTTP (WiFi)

Commands are sent as HTTP GET requests to `http://<IP>/js?json=<command>` ([HTTP reference](https://www.waveshare.com/wiki/RoArm-M3-S_Python_HTTP_Request_Communication)). Your computer must be on the same network as the arm:

- **AP mode:** connect your computer to the arm's own WiFi network. The arm's IP is `192.168.4.1`.
- **STA mode:** the arm joins your WiFi network. Its IP is shown on the arm's OLED screen.

## Usage

```bash
source .venv/bin/activate
python keyboard_controller.py --port /dev/ttyUSB0   # serial
python keyboard_controller.py --http                # HTTP, AP mode (192.168.4.1)
python keyboard_controller.py --http 192.168.1.50   # HTTP, STA mode (IP from the OLED)
```

| Option | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port of the arm |
| `--baud` | `115200` | Baud rate |
| `--http [IP]` | off | Use HTTP instead of serial. Without an IP, uses `192.168.4.1`. `--port` and `--baud` are ignored |
| `--dry-run` | off | Don't connect to the arm. Print the commands instead |

A small window opens. **The window must be in focus** for key presses to register. It uses `pygame` so that key presses and releases are detected reliably on both X11 and Wayland. The window shows each joint's angle, its position within its range, and its current speed. Each range bar is also a slider you can click or drag with the mouse (see [Mouse sliders](#mouse-sliders)).

At startup the controller turns the LED off, then asks the arm for its current joint angles (`{"T":105}`) so the first move continues from where the arm already is. If the arm doesn't answer, nothing is sent until you press a key.

### Keys

The same six key pairs are used in both modes:

| + | − | Joint mode (`T:101`) | Range (rad) | Axis mode (`T:123`) | Range |
|---|---|---|---|---|---|
| `Q` | `A` | 1 Base | -3.14 to 3.14 | 1 X (forward) | 100 to 450 mm |
| `W` | `S` | 2 Shoulder | -1.57 to 1.57 | 2 Y (left) | -350 to 350 mm |
| `E` | `D` | 3 Elbow | 0 to 3.14 | 3 Z (up) | -50 to 450 mm |
| `R` | `F` | 4 Wrist | -1.57 to 1.57 | 4 Pitch | -1.57 to 1.57 rad |
| `T` | `G` | 5 Roll | -3.14 to 3.14 | 5 Roll | -3.14 to 3.14 rad |
| `Y` | `H` | 6 Gripper | 1.08 to 3.14 | 6 Gripper | 1.08 to 3.14 rad |

| Key | Action |
|---|---|
| `Tab` | Switch between joint mode and axis mode |
| `0` | Move all joints to their home position (`{"T":100}`) |
| `P` | Re-read the arm's position (joint angles and gripper tip coordinates) |
| `L` | Turn the LED on or off (`{"T":114,"led":255}` / `{"T":114,"led":0}`) |
| `M` | Start recording / stop recording and save |
| `B` | Play a recording |
| `Space` or `Esc` | Stop playback |
| `Esc` | Quit (when not playing back) |

### How motion works

- **Joint mode, tap** a key: the joint moves one small step (2°).
- **Joint mode, hold** a key: after 0.25 s it starts moving continuously at 10°/s and speeds up by 40°/s every second, up to 90°/s. Releasing the key stops it.
- **Axis mode, hold** a key: the axis starts moving right away at a fixed speed (`spd` 5) and stops when you release the key. If you press the opposite key while holding one, the newest key wins; releasing it goes back to the one still held.
- **Axis mode, tap** a key: the axis moves for 0.1 s and stops. The distance depends on the arm's speed, so it isn't a fixed number of mm.
- **Mouse:** click or drag a slider to send that joint or axis to the value under the mouse (see below).
- Each joint stays within its range. Each axis is stopped at its range limit (see Axis mode below).
- If the window loses focus, all motion stops. Switching modes, quitting, or pressing `0` also stops any moving axis first.
- In HTTP mode, requests are sent one at a time in the background. If the arm answers more slowly than commands are generated, only the newest command for each joint or axis is sent, so the arm never falls behind. Axis stop commands are sent before anything else and retried up to 3 times if a request fails.
- The arm can't report whether its LED is on, so the controller turns it off at startup (`{"T":114,"led":0}`). After that, `L` always matches the real LED.

### Mouse sliders

Each bar under a joint or axis is a slider. Left to right covers the range in the [Keys](#keys) table. The blue handle shows where the joint or axis is now.

- **Click** anywhere on a bar: that joint or axis moves to the value under the mouse. A white outline marks where it is heading, and stays until it gets there.
- **Drag** along the bar: the target follows the mouse. You can drag past either end of the bar; the value stops at the end of the range.
- **Release** the mouse button: the joint or axis still finishes moving to the last target.
- **Joint mode:** the joint glides to the target at up to 90°/s (`MAX_SPEED`, the same top speed as holding a key), sending `T:101` targets along the way. Even a click at the far end of a bar doesn't make the servo jump at full speed.
- **Axis mode:** `T:123` can't go to a position, so the controller starts the axis toward the target (`T:123` with `cmd` 1 or 2) and stops it (`cmd` 0) when the position it reads back is within 2 mm (1° for pitch, roll and gripper) of the target, or has passed it. It reads the position 10 times a second, so the axis can stop a little past the target. If you drag the slider behind the moving axis, it turns around. If no position comes back from the arm for 0.5 s, the axis is stopped, because the controller can no longer tell when it arrives. In `--dry-run` the value just jumps to the target when you release the mouse.
- Pressing a key for a joint or axis takes over from its slider. Losing window focus, switching modes, pressing `0` or starting a playback stops all slider moves. Sliders can only be used for the current mode's controls, and do nothing during playback.
- Slider moves are recorded like key moves: the `T:101` targets in joint mode, the `T:123` start and stop commands in axis mode.

### Axis mode (T:123)

X points straight ahead of the arm, Y to its left and Z up (from the Waveshare docs). Positions are in mm. Pitch, roll and gripper are in radians.

- **Start and stop:** `T:123` doesn't take a position. Pressing a key sends one command that starts the axis moving, and releasing it sends one that stops it. Holding a key doesn't send anything else.
- **Speed:** every axis command uses the same `spd` (`AXIS_CMD_SPD`, default 5). The docs describe `spd` only as a coefficient (larger is faster, about 0 to 20), so raise or lower it to taste.
- **Position display and limits:** the arm doesn't report where it is while `T:123` moves it, so the controller reads the position (`{"T":105}`) 10 times a second while an axis moves, plus once after it stops. The window shows these values. When an axis reaches a limit in the table, the controller stops it. Because the position is only read every 0.1 s, the arm can pass a limit slightly before it stops, by about 0.1 s of motion. A key that would move an axis further past its limit is ignored, but the opposite key still works.
- **X/Y/Z limits:** the docs give none, so the limits in the table are cautious guesses. Widen them in the `AXES` list once you know your arm's workspace.
- **Re-reading the position:** moving in one mode changes the other mode's values (joint moves change the gripper tip coordinates, and the reverse). So when you switch modes, or press a key after the other mode or a playback has moved the arm, the controller first asks the arm for its position (`{"T":105}`). If the arm doesn't answer, it doesn't switch or move, and says so in the window. In `--dry-run` there is no arm, so the values of each mode are kept as they are, and the axis values don't change while an axis "moves".

## Recording and playback

**Record:** press `M`. The window shows `REC` with the elapsed time. Move the arm with the keys as usual: joint moves (`T:101`), axis start/stop commands (`T:123`), `0` (home) and `L` (LED) are all recorded with their timing. You can switch modes with `Tab` while recording. Press `M` again to stop. A save dialog opens; the file gets a `.json` extension if you don't type one. Cancelling the dialog discards the recording. Quitting while recording also discards it.

**Play back:** press `B` and choose a recording. The window then asks how many times to play it:

| Type | Result |
|---|---|
| nothing, or `1` | Play once |
| a number, e.g. `5` | Play 5 times |
| `0` | Loop forever |

Press `Enter` to start, or `Esc` to cancel.

Before each loop, the arm glides to the position it was in when the recording started (at 45°/s), waits 0.5 s, and then replays the commands with their original timing. The LED is also reset to its starting state if the recording switches it.

Axis moves are replayed as the same start and stop commands at the same times. How far the arm goes depends on that timing, so a replayed axis move can end up a little off from the original, and the error can add up over a long recording. Each loop starts again from the recorded joint angles, so it doesn't build up from one loop to the next. The axis limits still apply during playback, and any axis still moving at the end of a loop is stopped.

**Stop playback:** press `Space` or `Esc` at any time, including while the arm is gliding to the start position. The controller reads the arm's current angles and tells every joint to stay there, so the arm stops where it is, even in the middle of a home (`0`) move. If the arm doesn't answer (or in `--dry-run`), it finishes its last small step and stops. The movement keys are ignored during playback.

### Recording file format

```json
{
  "format": "roarm-m3-keyboard-recording",
  "version": 1,
  "duration": 4.21,
  "led": false,
  "start": {"1": 0.0, "2": 0.0, "3": 1.5708, "4": 0.0, "5": 0.0, "6": 3.1416},
  "commands": [
    {"t":0.31,"cmd":{"T":101,"joint":1,"rad":0.0349,"spd":0,"acc":0}},
    {"t":1.05,"cmd":{"T":114,"led":255}},
    {"t":1.80,"cmd":{"T":123,"m":1,"axis":3,"cmd":1,"spd":5}},
    {"t":2.64,"cmd":{"T":123,"m":1,"axis":3,"cmd":0,"spd":5}}
  ]
}
```

- `duration`: length of the recording in seconds.
- `led`: whether the controller thought the LED was on when recording started.
- `start`: joint angles (rad) when recording started, keyed by joint number.
- `commands`: each command that was sent, with `t` = seconds since the start. Only `T:100`, `T:101`, `T:114` and `T:123` (with `"m":1`) are allowed. `T:103` commands from recordings made with older versions of this controller also still play. Joint angles and `T:103` positions are clamped to their ranges when loaded, and `T:123` moves are stopped at the axis limits during playback, so an edited file can't drive the arm far past its limits.

## Tuning

These constants at the top of `keyboard_controller.py` control how the arm moves:

| Constant | Default | Meaning |
|---|---|---|
| `TAP_STEP` | 2° | Joint distance moved by a single tap |
| `HOLD_DELAY` | 0.25 s | How long a joint key must be held before continuous motion starts |
| `START_SPEED` | 10°/s | Joint speed when continuous motion starts |
| `ACCELERATION` | 40°/s² | How quickly the joint speed increases while a key is held |
| `MAX_SPEED` | 90°/s | Maximum joint speed |
| `SEND_HZ` | 25 | Maximum commands per second for each joint |
| `CMD_SPD`, `CMD_ACC` | 0, 0 | `spd` and `acc` values sent with `T:101` (0 = servo maximum) |
| `AXIS_CMD_SPD` | 5 | `spd` value sent with `T:123` (larger = faster; the docs suggest 0 to 20) |
| `AXIS_TAP_TIME` | 0.1 s | How long an axis moves for a single tap in axis mode |
| `AXIS_POLL_HZ` | 10 | Position reads per second while an axis moves (for the display and the limits) |
| `AXIS_POLL_AFTER_STOP` | 0.1 s | Wait after an axis stops before reading its final position |
| `SLIDER_TOLERANCE_MM` | 2 mm | How close X/Y/Z must get to a slider target before the axis stops |
| `SLIDER_TOLERANCE_RAD` | 1° | The same for pitch, roll and gripper |
| `SLIDER_FEEDBACK_TIMEOUT` | 0.5 s | Stop a slider-driven axis if no position is read back for this long |
| `LED_ON`, `LED_OFF` | 255, 0 | `led` values sent with `T:114` |
| `PLAYBACK_MOVE_SPEED` | 45°/s | Speed when gliding to a recording's start position |
| `PLAYBACK_SETTLE` | 0.5 s | Pause at the start position before each loop |
| `DEFAULT_HTTP_HOST` | `192.168.4.1` | IP used by `--http` when none is given |
| `HTTP_TIMEOUT` | 1.0 s | How long to wait for each HTTP request |

To change a key, or to flip a direction, edit the `JOINTS` list (joint mode) or the `AXES` list (axis mode). The `AXES` list also holds the X/Y/Z limits.

## Troubleshooting

- **Keys do nothing:** click the controller window so it has focus.
- **Clicking a slider does nothing:** click on the bar itself, not the text above it. Only the current mode's sliders work, and not during playback. If the window says the position is unknown, the arm didn't answer the position request.
- **"Stopped the slider move: no position feedback from the arm":** in axis mode the controller has to read the arm's position to know when to stop. Check the connection (or the red HTTP error) and try again.
- **An axis slider stops short of or past the target:** lower `SLIDER_TOLERANCE_MM` / `SLIDER_TOLERANCE_RAD` to stop closer, lower `AXIS_CMD_SPD` to overshoot less, or raise `AXIS_POLL_HZ`.
- **`Permission denied` on the serial port:** add your user to the `dialout` group (see Setup).
- **Arm doesn't respond / "No feedback from arm":** check the port name (`ls /dev/ttyUSB* /dev/ttyACM*`) and try a different `--baud`.
- **"HTTP request failed" (shown in red):** check that your computer is on the same WiFi network as the arm and that the IP matches the one on the OLED screen. Opening `http://<IP>` in a browser should show the arm's web page.
- **"No file dialog available":** install `zenity` or `kdialog` (see Setup). If the save dialog fails after a recording, the recording is saved as `recording-<date>-<time>.json` in the current folder instead.
- **A joint moves the wrong way:** swap its `key_minus` and `key_plus` in `JOINTS` (or `AXES` for axis mode).
- **"Can't switch modes" / "position is unknown":** the arm didn't answer the position request (`{"T":105}`). Check the connection and try again.
- **Axis mode is too slow or too fast:** change `AXIS_CMD_SPD`.
- **Axis mode keeps moving after you let go:** the stop command didn't reach the arm (look for a red HTTP error). Tap the same key again: that sends a new start and stop. Quitting also sends a stop for any axis the controller thinks is moving.
- **Axis mode stops at a limit too early:** widen the X/Y/Z ranges in `AXES`.
- **Axis mode goes too far past a limit:** lower `AXIS_CMD_SPD`, or raise `AXIS_POLL_HZ` so the position is read more often.
