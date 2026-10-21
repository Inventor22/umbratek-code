# Capture 2026-10-21: UTRA 6-1000 at rest and in small moves

Recorded 02:37 to 02:41 local time with the owner present, by the capture tools that were then in the jog panel
session (`capture.py`, `line_check.mjs`, `settings_probe.py`, to be moved into the repo). Every move went through
the jog panel's own API, so joint moves were collision-checked by the panel; the straight tool lines were checked
beforehand with the 3D view's inverse kinematics and collision model.

## Conditions
- Arm on its cart, nothing on the flange: tool offset and payload are both zero in the controller.
- Controller firmware 739c89bb82 (2021-12-18), hardware 0001110001 / x86 J1900; tool GPIO be675f3-1013,
  controller GPIO d4265e8-1217; servos 98bb7af2f850. Linked over the controller's Wi-Fi (NTRO_43) at 100% signal.
- Collision sensitivity 0 and teach (freedrive) sensitivity 0, as set. Gravity direction [0, 0, -1].
- Start pose (deg): 36.109, 28.528, 138.785, 108.434, 45.766, 64.623; the controller reports the TCP there at
  x 308.38, y -13.28, z 477.29 mm, roll/pitch/yaw 92.912, -63.32, 79.26 deg, and our model agrees on the position.

## Files
| File | What it is |
| ---- | ---------- |
| `status10hz.bin` | Every frame of the 10 Hz status stream (port 30001): repeated `<qqH` (host wall-clock ns, host monotonic ns, frame length) then the frame. Frame: `<HBBBIIBBH` (length, axis, motion status, motion mode, brake mask, enable mask, error, warning, queued commands), then 6 x f32 joints [rad], 6 x f32 TCP pose [mm, rad], 6 x f32 joint "tau" (controller units; see below). |
| `config.bin` | The config stream (port 30003), same wrapper. Frame `<H19fBB`: linear max acc and jerk, rotational max acc and jerk, joint max acc and jerk, TCP offset x6, TCP load x4, gravity direction x3, collision and teach sensitivity. Constant throughout. |
| `marks.json` | The start pose and a timestamped mark (host monotonic ns) at the start of each phase and move. |
| `commands.json` | Every request to the panel: path, body, send and reply times (host monotonic ns), reply. |
| `panel_status.jsonl` | What the panel reported, about 10 times a second (rounded: 0.01 mm, 0.001 deg). |
| `panel_config.json` | The panel's config, including the collision model and drag limits used for the checks. |
| `settings.json`, `arm_status.txt` | Read-only identity, motion settings and every joint servo's registers (versions, limits, PID gains, temperatures, bus voltage), with the motors off. |
| `summary.json` | What `capture.py analyze` makes of it all (below). |

## Sequence
60 s at rest (motors on); each joint out and back both ways at two speeds (J1 +/-15 deg at 10 and 30 deg/s; J2, J3
+/-10 deg at 10 and 20; J4, J5 +/-20 deg at 15 and 45; J6 +/-30 deg at 15 and 60); straight tool lines with the
controller's own linear moves, out and back in 50 mm clicks at 50 and 100 mm/s (X +100 and -50 mm, Y and Z
+/-100 mm); a STOP while J1 turned +30 deg at 30 deg/s, then Enable; back to the start pose; 15 s at rest.

## What it shows
- **Only 10 Hz state.** The SDK's 100 Hz port (30002) refuses connections on this firmware; the 10 Hz stream ran at
  10.01 Hz, intervals 80 to 109 ms (99% within 89 to 103), no gaps over the whole capture. A second client on port
  30001 works alongside the panel's. Command to first visible motion: about 110 ms (median), frame-quantised.
- **Still at rest.** Joints vary by under 0.005 deg; the TCP stays within 0.014 mm.
- **"tau" tracks gravity and motion.** At rest J3 reads -25.9 (sd 0.2) and J2 5.5 (sd 0.08); J3's reading moves
  with its angle (-18.4 at +10 deg, -27.2 at -10 deg); moving J1 at 30 deg/s reads up to +/-25. The SDK calls it the
  joints' "actual current"; its units are unconfirmed (the servo registers' TAU_LIMIT_MAX is 5.93 for J1 and J2, 2.72
  for J3 and 2.54 for the wrists, so the status value is probably scaled, perhaps to joint torque).
- **Straight lines are straight.** Within 0.21 mm of the line over 100 mm, orientation within 0.13 deg. Asked for
  100 mm/s, 50 mm clicks peak at about 90 mm/s (200 mm/s^2 and 2000 mm/s^3 limits). From this pose, -X runs into a
  singularity about 95 mm out.
- **STOP is abrupt.** From 30 deg/s, J1 travelled 0.21 deg (about 1.1 mm at the flange) after the request and was
  still within 300 ms (frame-quantised).
- **Enabling shifts the arm.** Switching the motors on moved the TCP 0.53 mm and 0.46 mm (J3 about 0.06 deg) on
  the two occasions.
- **Joint speeds hold.** Every joint reached the speed asked (within the 10 Hz sampling), and settled within 0.2 deg.
