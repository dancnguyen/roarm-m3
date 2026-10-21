# RoArm-M3 Keyboard Controller

Control each joint of a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3) from the keyboard over a USB serial connection.

Every move is sent as a `{"T":101}` single-joint command ([command reference](https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control)):

```json
{"T":101,"joint":1,"rad":0.5,"spd":0,"acc":0}
```

## Setup

Requires Python 3.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The arm usually shows up as `/dev/ttyUSB0` (or `/dev/ttyACM0`). Your user needs access to the serial port. On most Linux distributions:

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

## Usage

```bash
source .venv/bin/activate
python keyboard_controller.py --port /dev/ttyUSB0
```

| Option | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port of the arm |
| `--baud` | `115200` | Baud rate |
| `--dry-run` | off | Don't connect to the arm. Print the commands instead |

A small window opens. **The window must be in focus** for key presses to register. It uses `pygame` so that key presses and releases are detected reliably on both X11 and Wayland. The window shows each joint's angle, its position within its range, and its current speed.

At startup the controller asks the arm for its current joint angles (`{"T":105}`) so the first move continues from where the arm already is. If the arm doesn't answer, nothing is sent until you press a key.

### Keys

| Joint | + | − | Range (rad) |
|---|---|---|---|
| 1 Base | `Q` | `A` | -3.14 to 3.14 |
| 2 Shoulder | `W` | `S` | -1.57 to 1.57 |
| 3 Elbow | `E` | `D` | 0 to 3.14 |
| 4 Wrist | `R` | `F` | -1.57 to 1.57 |
| 5 Roll | `T` | `G` | -3.14 to 3.14 |
| 6 Gripper | `Y` | `H` | 1.08 to 3.14 |

| Key | Action |
|---|---|
| `0` | Move all joints to their home position (`{"T":100}`) |
| `P` | Re-read the joint angles from the arm |
| `Esc` | Quit |

### How motion works

- **Tap** a key: the joint moves one small step (2°).
- **Hold** a key: after 0.25 s the joint starts moving continuously at 10°/s and speeds up by 40°/s every second, up to 90°/s. Releasing the key stops it.
- Each joint stays within its range.
- If the window loses focus, all motion stops.

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
| `CMD_SPD`, `CMD_ACC` | 0, 0 | `spd` and `acc` values sent with `T:101` (0 = servo maximum) |

To change a key, or to flip a joint's direction, edit the `JOINTS` list.

## Troubleshooting

- **Keys do nothing:** click the controller window so it has focus.
- **`Permission denied` on the serial port:** add your user to the `dialout` group (see Setup).
- **Arm doesn't respond / "No feedback from arm":** check the port name (`ls /dev/ttyUSB* /dev/ttyACM*`) and try a different `--baud`.
- **A joint moves the wrong way:** swap its `key_minus` and `key_plus` in `JOINTS`.
