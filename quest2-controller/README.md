# RoArm-M3 Quest 2 Controller

Control a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3) with a Meta Quest 2 controller. Hold the grip button and the arm's gripper tip follows your hand: move the controller to move the tip, tilt and roll it to tilt and roll the tip, and pull the trigger to close the gripper. Movements can be recorded and played back once, a set number of times, or forever. Commands go to the arm over USB serial.

## How it works

```
Quest 2 (Quest Browser, WebXR page)  ──WiFi, WebSocket──▶  quest2_controller.py  ──USB serial──▶  RoArm-M3
        controller pose + buttons                        turns them into {"T":1041} commands
```

- `quest2_controller.py` runs a small HTTPS web server on your computer.
- You open its page in the **Quest Browser**. Nothing is installed on the headset. The page is a WebXR app that sends the controllers' positions, orientations and buttons to the computer about 70 times a second.
- The computer turns them into `T:1041` commands ([command reference](https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control)). `T:1041` sets the gripper tip's position (X/Y/Z in mm), pitch, roll and gripper angle, and the arm works out the joint angles itself:
  ```json
  {"T":1041,"x":235,"y":0,"z":234,"t":0,"r":0,"g":3.14}
  ```
  `T:1041` moves to each target at full speed. So the controller sends small steps (up to 25 a second) and limits the speed itself (see [Tuning](#tuning)).
- WebXR only works on secure (`https`) pages. On first run the program makes a self-signed certificate in `certs/`. The Quest Browser warns about it once (see [On the Quest](#on-the-quest)).

## Setup

Requires Python 3.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` installs:

| Package | Used for |
|---|---|
| `aiohttp` | Web server and WebSocket for the Quest page |
| `cryptography` | Making the self-signed HTTPS certificate |
| `pyserial` | Serial connection to the arm |

### Serial (USB)

The arm usually shows up as `/dev/ttyUSB0` (or `/dev/ttyACM0`). Your user needs access to the serial port. On most Linux distributions:

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

### Network

- The Quest and the computer must be on the same network (e.g. the same WiFi router).
- The page is served on port `8443`. If your computer has a firewall, allow it, e.g. on Ubuntu: `sudo ufw allow 8443/tcp`.
- No WiFi? See [USB instead of WiFi](#usb-instead-of-wifi-adb).

## Usage

```bash
source .venv/bin/activate
python quest2_controller.py --port /dev/ttyUSB0
python quest2_controller.py --dry-run            # no arm: print the commands instead
```

The program reads the arm's position (`{"T":105}`), then prints the address to open on the Quest, e.g.:

```
Open this address in the Quest Browser:
  https://192.168.1.20:8443
```

| Option | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port of the arm |
| `--baud` | `115200` | Baud rate |
| `--dry-run` | off | Don't connect to the arm. Print the commands instead |
| `--hand` | `right` | Which controller moves the arm (`right` or `left`) |
| `--scale` | `1.0` | Arm mm per controller mm. `0.5` = the arm moves half as far as your hand, for finer work |
| `--arm-yaw DEG` | `0` | Which way the arm faces compared to you (see [Where to stand](#where-to-stand)) |
| `--gripper` | `analog` | `analog`: the trigger sets how far the gripper closes. `toggle`: each trigger pull opens or closes it |
| `--listen-port` | `8443` | Port for the web page |
| `--host` | `0.0.0.0` | Network address to listen on (all by default) |
| `--no-tls` | off | Serve plain `http`. Only for [USB with adb](#usb-instead-of-wifi-adb) |
| `--cert`, `--key` | none | Use your own certificate and key instead of the self-signed one |

Press `Ctrl+C` to quit.

### On the Quest

1. Open the **Quest Browser** and type in the address the program printed (`https://<computer IP>:8443`).
2. The first time, the browser says the connection is not private, because the certificate is self-signed. Choose **Advanced**, then **Proceed to ... (unsafe)**. It asks again only if the certificate changes, e.g. when your computer's IP address changes.
3. The page shows **Server: connected** and the arm's position.
4. Press **Start (passthrough)**. You see the room through the headset cameras, so you can watch the arm. Allow the permission prompt if one appears. **Start (VR)** works too, but then you can't see the arm.
5. Each controller has a small marker on it. The one on the controller that moves the arm changes colour:

| Colour | Meaning |
|---|---|
| Grey | Idle. Hold the grip to move the arm |
| Blue | Reading the arm's position (just after you squeeze, or before recording or playing) |
| Green | The arm is following the controller |
| Orange | You are pushing past a limit. The arm waits at the edge |
| Purple | Playing a recording (both markers) |
| Red | Not connected to the computer |

The marker on the other controller is dark grey, and turns red while recording.

To leave, press the Meta (Oculus) button and quit the immersive view. The page's status panel and the computer's terminal show what is happening.

### Controls

The right controller moves the arm, and the left one records and plays (swapped with `--hand left`: then the left controller's X and Y do what A and B do here, and the right controller's A and B record and play).

| Button | Action |
|---|---|
| Right **grip** (middle finger, hold) | The arm follows the controller while held. Release to freeze the arm where it is |
| Right **trigger** (index finger) | Gripper. `analog`: pull to close, further = tighter. `toggle`: each pull opens or closes |
| **A** | Send the arm to its home position (`{"T":100}`) |
| **B** | Turn pitch/roll following on or off. When off, only the position follows your hand |
| **X** | Start recording / stop recording and save (see [Recording and playback](#recording-and-playback)) |
| **Y** | Play the selected recording / stop playing |

The controller vibrates briefly when the arm starts following (grip), when it stops (release), repeatedly while you push past a limit, and with a long buzz when something can't be done (see the status message). The left controller gives a long pulse when recording starts or stops, and a short one when playback starts or ends.

### How motion works

- **Hold to move ("clutch"):** nothing moves unless you hold the grip button. When you squeeze, the arm stays where it is, and from then on it copies how your hand moves *from that point*. When you run out of reach, release the grip, move your hand back, and squeeze again to carry on, like lifting a mouse off the desk. The trigger and gripper also only work while you hold the grip.
- **Directions:** "forward" is the direction you are looking (level with the floor) at the moment you squeeze the grip. With the arm in front of you and facing the same way as you:
  - push the controller forward → the tip moves forward (+X)
  - move it left → the tip moves left (+Y)
  - raise it → the tip goes up (+Z)
  - tilt its nose down → the tip tilts down (pitch)
  - roll it clockwise → the tip rolls clockwise (roll)
- **Scale and speed:** by default the tip moves 1 mm per mm of hand motion (`--scale`). It never moves faster than 250 mm/s, or turns faster than 120°/s, however quickly you move. A light filter smooths out hand tremor.
- **Re-reading the arm:** each time you squeeze, the program reads the arm's real position (`{"T":105}`), so it continues correctly even if something else moved the arm. If the arm is within 20 mm of where it was last sent, the last target is used instead, so a loaded arm that sags a little doesn't creep downwards every time you squeeze. If the arm doesn't answer, it doesn't move, and the controller buzzes.
- **Gripper takeover (`analog`):** when you squeeze the grip, the gripper stays as it is until the trigger reaches the gripper's current position. Then the trigger takes over (you feel a short tick). So if the arm is holding something, it won't drop it just because your trigger finger was relaxed when you squeezed. The gripper is never opened by the program on its own.
- **Workspace limits:** the tip is kept inside a ring around the base, 100 to 450 mm from the base axis, at a height of -50 to 450 mm, and within 120° either side of straight ahead. Pitch is kept within ±1.57 rad, roll within ±3.14 rad and the gripper within 1.08 to 3.14 rad. If you push past a limit, the arm slides along the edge, the marker turns orange and the controller pulses. The X/Y/Z limits are cautious guesses, because the Waveshare docs give none. Widen them in `quest2_controller.py` once you know your arm's reach (see [Tuning](#tuning)).
- **If the arm is outside the limits** when you squeeze (e.g. moved there with the Waveshare web page), it won't follow. Press **A** to send it home first.
- **Safety stops:** the arm freezes where it is if the controller stops being tracked, or no update arrives for 0.3 s (headset removed, WiFi dropout, browser closed). After that you have to release and squeeze the grip again. Only one headset controls the arm at a time: the one that most recently pressed Start. The page can also be open in other browsers, e.g. on the computer, to watch the status and to record and play (see [Recording and playback](#recording-and-playback)).
- **Home (A):** sends `{"T":100}`. The arm moves home on its own. Squeezing the grip is ignored for 3 s while it gets there.

### Where to stand

The easiest setup: stand **behind the arm, facing the same way it faces**. Then your forward is the arm's forward, and your left is its left.

If you'd rather stand somewhere else, tell the program which way the arm faces compared to you, in degrees counterclockwise (seen from above):

| You stand | `--arm-yaw` |
|---|---|
| Behind the arm, facing the same way | `0` (default) |
| In front of the arm, facing it | `180` |
| At the arm's left side, facing it | `90` |
| At the arm's right side, facing it | `-90` |

For example, with `--arm-yaw 180` pushing the controller away from you moves the tip along the arm's -X, which is away from you. As seen by you, the tip always moves the same way as your hand.

### USB instead of WiFi (adb)

If the Quest is in [developer mode](https://developers.meta.com/horizon/documentation/native/android/mobile-device-setup/) and connected to the computer with a USB cable, you can skip the network and the certificate warning:

```bash
adb reverse tcp:8443 tcp:8443
python quest2_controller.py --port /dev/ttyUSB0 --no-tls
```

Then open `http://localhost:8443` in the Quest Browser. Browsers treat `localhost` as secure, so WebXR works over plain `http`. Install `adb` with e.g. `sudo apt install adb`.

## Recording and playback

**Record:** press **X** (or **Start recording** on the page). The other controller's marker turns red and the page shows `REC` with the elapsed time. Move the arm as usual: every command sent to the arm is recorded with its timing, including `A` (home). Press **X** again to stop. The recording is saved straight away in the `recordings/` folder as `recording-<date>-<time>.json`, and becomes the selected recording. If nothing moved, nothing is saved. Quitting the program while recording also saves it.

**Play:** press **Y** to play the selected recording. Before entering the immersive view (or on any other browser), the page's **Recordings** panel lists the files in `recordings/`, newest first. Pick one, set **Loops**, and press **Play**, or press **Y** later in the headset. The selection and loop count are shared, so a choice made on the computer is what **Y** plays.

| Loops | Result |
|---|---|
| `1` | Play once (default) |
| a number, e.g. `5` | Play 5 times |
| `0` | Loop forever |

Before each loop, the arm glides to where the tip was when the recording started (at up to 100 mm/s and 45°/s), waits 0.5 s, and then replays the commands with their original timing. The glide goes around the base, not straight across it. The recorded moves still go through the speed limit and workspace limits, so an edited file can't make the arm jump or leave the workspace.

**Stop playback:** press **Y** again, press **Stop** on the page, or squeeze the right grip. The arm stops within one small step. If it was in the middle of a home move (`T:100`), it finishes that. After squeezing the grip to stop, release and squeeze it again to start following.

While playing, the other buttons are ignored. You can't record during playback, or play while recording.

Playback carries on if the headset disconnects or is taken off. Stop it from the page, or quit the program.

To keep a recording, rename it in `recordings/` (keep the `.json` extension). Delete files there to remove them from the list, then press **Refresh list**. Recordings from the keyboard controller can't be played: they use joint commands, while this one records gripper tip positions.

### Using the page on the computer

Open the same address (`https://<computer IP>:8443`, or `https://localhost:8443` on the computer itself) in a browser on the computer, and accept the certificate warning. The page shows the arm's status and the **Recordings** panel, so you can pick, play and stop recordings from there, with or without the headset. Opening it there doesn't affect the headset's control.

### Recording file format

```json
{
  "format": "roarm-m3-quest2-recording",
  "version": 1,
  "duration": 6.12,
  "start": {"x": 345.0, "y": 0.0, "z": 207.0, "t": 0.0, "r": 0.0, "g": 3.14},
  "commands": [
    {"t":0.374,"cmd":{"T":1041,"x":345.7,"y":0.0,"z":207.0,"t":0.0,"r":0.0,"g":3.14}},
    {"t":0.418,"cmd":{"T":1041,"x":349.4,"y":0.0,"z":207.0,"t":0.0,"r":0.0,"g":3.14}},
    {"t":4.90,"cmd":{"T":100}}
  ]
}
```

- `duration`: length of the recording in seconds.
- `start`: gripper tip pose when recording started: `x`, `y`, `z` in mm, and pitch `t`, roll `r` and gripper `g` in radians.
- `commands`: each command that was sent, with `t` = seconds since the start. Only `T:1041` and `T:100` are allowed. Positions and angles are clamped to the workspace and angle ranges when the file is loaded.

## Tuning

These constants at the top of `quest2_controller.py` control how the arm moves:

| Constant | Default | Meaning |
|---|---|---|
| `DEFAULT_SCALE` | 1.0 | Default for `--scale` |
| `SEND_HZ` | 25 | Maximum `T:1041` commands per second |
| `MAX_LINEAR_SPEED` | 250 mm/s | Fastest the tip moves |
| `MAX_ANGULAR_SPEED` | 120°/s | Fastest pitch, roll and gripper turn |
| `SMOOTHING` | 0.06 s | Smoothing of hand motion. Larger = steadier but laggier. `0` = off |
| `POSE_TIMEOUT` | 0.3 s | Freeze the arm if no controller update arrives for this long |
| `GRIP_ON`, `GRIP_OFF` | 0.6, 0.3 | How far the grip button must be pressed to start following, and released to stop |
| `MIN_RADIUS`, `MAX_RADIUS` | 100, 450 mm | Closest to and furthest from the base axis the tip may go |
| `Z_MIN`, `Z_MAX` | -50, 450 mm | Lowest and highest the tip may go |
| `BASE_ANGLE_LIMIT` | 120° | How far either side of straight ahead the tip may go |
| `PITCH_RANGE`, `ROLL_RANGE`, `GRIPPER_RANGE` | ±1.57, ±3.14, 1.08 to 3.14 rad | Angle ranges |
| `GRIPPER_OPEN`, `GRIPPER_CLOSED` | 1.57, 3.14 rad | Gripper angle with the trigger released and fully pulled. Lower `GRIPPER_OPEN` (down to 1.08) to open wider |
| `GRIPPER_TAKEOVER` | 0.15 rad | How close the trigger must get to the gripper's position before it takes over |
| `PITCH_SIGN`, `ROLL_SIGN` | -1, +1 | Flip (`-1` ↔ `+1`) if pitch or roll goes the wrong way. `0` ignores it |
| `HOME_WAIT` | 3 s | How long squeezing is ignored after `A` (home) |
| `RESYNC_TOLERANCE_MM`, `RESYNC_TOLERANCE_RAD` | 20 mm, 0.2 rad | When squeezing, keep the last target if the arm reports a position this close to it |
| `PLAYBACK_LINEAR_SPEED`, `PLAYBACK_ANGULAR_SPEED` | 100 mm/s, 45°/s | Speed when gliding to a recording's start position |
| `PLAYBACK_SETTLE` | 0.5 s | Pause at the start position before each loop |
| `HAPTIC_*` | | Strength (0 to 1) and length (ms) of each vibration |
| `DEFAULT_LISTEN_PORT` | 8443 | Default for `--listen-port` |

## Troubleshooting

- **The page doesn't load on the Quest:** check that the Quest and computer are on the same network, that you typed `https://` and the port `:8443`, and that the firewall allows port 8443. Try opening the address in a browser on the computer too.
- **"Your connection is not private":** expected the first time. Choose **Advanced → Proceed**. If you'd rather avoid it, use [adb](#usb-instead-of-wifi-adb), or pass your own trusted certificate with `--cert` and `--key`.
- **The Start buttons are greyed out:** the page isn't secure (it must be `https://`, or `http://localhost` with adb), or the browser isn't the Quest Browser.
- **Server: disconnected:** the program isn't running, or the network dropped. The page reconnects by itself.
- **Squeezing the grip does nothing (long buzz):** read the message on the page or in the terminal. Usually the arm didn't answer the position request (check the USB cable and `--port`), it is outside the workspace limits (press **A**), or it is still moving home.
- **The arm moves the wrong way:** stand behind the arm facing the same way, or set `--arm-yaw` (see [Where to stand](#where-to-stand)). Remember that "forward" is where you were looking when you squeezed the grip.
- **Pitch or roll goes the wrong way:** flip `PITCH_SIGN` or `ROLL_SIGN`. Press **B** to stop following pitch and roll altogether.
- **The arm is jerky:** lower `MAX_LINEAR_SPEED`, raise `SMOOTHING` a little, or use `--scale 0.5`. A weak WiFi signal also causes stutter.
- **The arm stops short of where you want it (orange marker):** widen the workspace limits (`MAX_RADIUS`, `Z_MIN`, `Z_MAX`, `BASE_ANGLE_LIMIT`).
- **The gripper doesn't respond to the trigger:** with `analog`, move the trigger to roughly where the gripper is first (see *Gripper takeover*). The page's status message says which way.
- **`Permission denied` on the serial port:** add your user to the `dialout` group (see Setup).
- **"The arm didn't answer yet" at startup:** check the port name (`ls /dev/ttyUSB* /dev/ttyACM*`) and that the arm is powered. The program asks again each time you squeeze the grip.
- **Certificate keeps changing:** your computer's IP address changes, e.g. from DHCP. Give it a fixed address in your router, or use adb.

- **X / Y do nothing:** the other controller must be on and tracked. Check the status message: you can't record during playback, or play while recording.
- **"No recording to play":** record one first, or pick one in the page's **Recordings** panel.
- **A recording is greyed out in the list:** the file can't be played. The reason is shown next to it (e.g. it isn't a recording from this program).
- **A replayed move isn't quite where it was:** recordings replay the targets that were sent. If the arm was carrying a load when recording and not when playing (or the reverse), it can sag differently.

## Files

| File | Contents |
|---|---|
| `quest2_controller.py` | The program: web server, motion control and serial link |
| `web/index.html`, `web/app.js` | The WebXR page that runs in the Quest Browser |
| `requirements.txt` | Python packages |
| `certs/` | The self-signed certificate and its private key, made on first run. Delete the folder to make a new one |
| `recordings/` | Saved recordings, made when you save the first one |
