# RoArm-M3 Keyboard Controller

Control each joint of a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3) from the keyboard, over a USB serial connection or over WiFi with HTTP requests. Movements can be recorded to a JSON file and played back once, a set number of times, or forever.

There are two control modes ([command reference](https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control)). `Tab` switches between them:

- **Joint mode:** each key pair turns one joint, sent as a `{"T":101}` single-joint command:
  ```json
  {"T":101,"joint":1,"rad":0.5,"spd":0,"acc":0}
  ```
- **Axis mode:** each key pair moves the gripper tip along one axis (X, Y, Z, pitch, roll, gripper), sent as a `{"T":103}` single-axis command. The arm works out the joint angles itself:
  ```json
  {"T":103,"axis":1,"pos":350,"spd":0.25}
  ```

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

A small window opens. **The window must be in focus** for key presses to register. It uses `pygame` so that key presses and releases are detected reliably on both X11 and Wayland. The window shows each joint's angle, its position within its range, and its current speed.

At startup the controller turns the LED off, then asks the arm for its current joint angles (`{"T":105}`) so the first move continues from where the arm already is. If the arm doesn't answer, nothing is sent until you press a key.

### Keys

The same six key pairs are used in both modes:

| + | − | Joint mode (`T:101`) | Range (rad) | Axis mode (`T:103`) | Range |
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

- **Tap** a key: the joint or axis moves one small step (2°, or 5 mm for X/Y/Z).
- **Hold** a key: after 0.25 s it starts moving continuously at 10°/s and speeds up by 40°/s every second, up to 90°/s. For X/Y/Z: 20 mm/s, speeding up by 60 mm/s every second, up to 100 mm/s. Releasing the key stops it.
- Each joint and axis stays within its range.
- If the window loses focus, all motion stops.
- In HTTP mode, requests are sent one at a time in the background. If the arm answers more slowly than commands are generated, only the newest target for each joint is sent, so the arm never falls behind.
- The arm can't report whether its LED is on, so the controller turns it off at startup (`{"T":114,"led":0}`). After that, `L` always matches the real LED.

### Axis mode (T:103)

X points straight ahead of the arm, Y to its left and Z up (from the Waveshare docs). Positions are in mm. Pitch, roll and gripper are in radians.

- **X/Y/Z limits:** the docs give none, so the limits in the table are cautious guesses. Widen them in the `AXES` list once you know your arm's workspace. If the arm is already outside a limit (for example X behind the base), keys can move it back toward the range but not further out.
- **Fewer, larger steps:** the docs say `T:103` makes the arm finish each move before it reads the next command. While a key is held, axis mode sends at most 5 commands per second per axis (`AXIS_SEND_HZ`) instead of the 25 per second used for joints.
- **Re-reading the position:** moving in one mode changes the other mode's values (joint moves change the gripper tip coordinates, and the reverse). So when you switch modes, or press a key after the other mode or a playback has moved the arm, the controller first asks the arm for its position (`{"T":105}`). If the arm doesn't answer, it doesn't switch or move, and says so in the window. In `--dry-run` there is no arm, so the values of each mode are kept as they are.

## Recording and playback

**Record:** press `M`. The window shows `REC` with the elapsed time. Move the arm with the keys as usual: joint moves (`T:101`), axis moves (`T:103`), `0` (home) and `L` (LED) are all recorded with their timing. You can switch modes with `Tab` while recording. Press `M` again to stop. A save dialog opens; the file gets a `.json` extension if you don't type one. Cancelling the dialog discards the recording. Quitting while recording also discards it.

**Play back:** press `B` and choose a recording. The window then asks how many times to play it:

| Type | Result |
|---|---|
| nothing, or `1` | Play once |
| a number, e.g. `5` | Play 5 times |
| `0` | Loop forever |

Press `Enter` to start, or `Esc` to cancel.

Before each loop, the arm glides to the position it was in when the recording started (at 45°/s), waits 0.5 s, and then replays the commands with their original timing. The LED is also reset to its starting state if the recording switches it.

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
    {"t":1.05,"cmd":{"T":114,"led":255}}
  ]
}
```

- `duration`: length of the recording in seconds.
- `led`: whether the controller thought the LED was on when recording started.
- `start`: joint angles (rad) when recording started, keyed by joint number.
- `commands`: each command that was sent, with `t` = seconds since the start. Only `T:100`, `T:101`, `T:103` and `T:114` are allowed. Joint angles and axis positions are clamped to their ranges when loaded, so an edited file can't drive the arm past its limits.

## Tuning

These constants at the top of `keyboard_controller.py` control how the arm moves:

| Constant | Default | Meaning |
|---|---|---|
| `TAP_STEP` | 2° | Distance moved by a single tap |
| `HOLD_DELAY` | 0.25 s | How long a key must be held before continuous motion starts |
| `START_SPEED` | 10°/s | Speed when continuous motion starts |
| `ACCELERATION` | 40°/s² | How quickly the speed increases while a key is held |
| `MAX_SPEED` | 90°/s | Maximum speed |
| `SEND_HZ` | 25 | Maximum commands per second for each joint |
| `LINEAR_TAP_STEP` | 5 mm | X/Y/Z distance moved by a single tap |
| `LINEAR_START_SPEED` | 20 mm/s | X/Y/Z speed when continuous motion starts |
| `LINEAR_ACCELERATION` | 60 mm/s² | How quickly the X/Y/Z speed increases while a key is held |
| `LINEAR_MAX_SPEED` | 100 mm/s | Maximum X/Y/Z speed |
| `AXIS_SEND_HZ` | 5 | Maximum `T:103` commands per second for each axis |
| `CMD_SPD`, `CMD_ACC` | 0, 0 | `spd` and `acc` values sent with `T:101` (0 = servo maximum) |
| `AXIS_CMD_SPD` | 0.25 | `spd` value sent with `T:103` (larger = faster; 0.25 is Waveshare's example) |
| `LED_ON`, `LED_OFF` | 255, 0 | `led` values sent with `T:114` |
| `PLAYBACK_MOVE_SPEED` | 45°/s | Speed when gliding to a recording's start position |
| `PLAYBACK_SETTLE` | 0.5 s | Pause at the start position before each loop |
| `DEFAULT_HTTP_HOST` | `192.168.4.1` | IP used by `--http` when none is given |
| `HTTP_TIMEOUT` | 1.0 s | How long to wait for each HTTP request |

To change a key, or to flip a direction, edit the `JOINTS` list (joint mode) or the `AXES` list (axis mode). The `AXES` list also holds the X/Y/Z limits.

## Troubleshooting

- **Keys do nothing:** click the controller window so it has focus.
- **`Permission denied` on the serial port:** add your user to the `dialout` group (see Setup).
- **Arm doesn't respond / "No feedback from arm":** check the port name (`ls /dev/ttyUSB* /dev/ttyACM*`) and try a different `--baud`.
- **"HTTP request failed" (shown in red):** check that your computer is on the same WiFi network as the arm and that the IP matches the one on the OLED screen. Opening `http://<IP>` in a browser should show the arm's web page.
- **"No file dialog available":** install `zenity` or `kdialog` (see Setup). If the save dialog fails after a recording, the recording is saved as `recording-<date>-<time>.json` in the current folder instead.
- **A joint moves the wrong way:** swap its `key_minus` and `key_plus` in `JOINTS` (or `AXES` for axis mode).
- **"Can't switch modes" / "position is unknown":** the arm didn't answer the position request (`{"T":105}`). Check the connection and try again.
- **Axis mode lags (the arm keeps moving after you let go):** the arm is still working through earlier `T:103` moves. Raise `AXIS_CMD_SPD` so each move finishes sooner, or lower `AXIS_SEND_HZ`.
- **Axis mode stops at a limit too early:** widen the X/Y/Z ranges in `AXES`.
