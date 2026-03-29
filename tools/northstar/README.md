# Northstar Setup

This folder gives you a practical starting point for running Northstar against your existing Limelights.

## Assumed topology

- Limelights stay on the robot and serve MJPEG video
- A separate coprocessor runs one Northstar process per camera
- The roboRIO remains the NetworkTables server

## Files

- `config-left.json`
- `config-right.json`
- `config-side.json`
- `run-northstar-macos.sh`

## Before you run

1. Clone `RobotCode2026Public` somewhere on the coprocessor.
2. Copy these config files to that clone's `northstar/` folder, or point `--config` at them directly.
3. Create one calibration file per camera:
   - `calibration-left.yml`
   - `calibration-right.yml`
   - `calibration-side.yml`
4. Edit each config file's `server_ip` to your roboRIO address.
5. If `.local` hostnames are unreliable on your network, replace the Limelight URLs with fixed IPs.

## Calibration workflow

Northstar listens for calibration commands on:

- `/northstar_0/calibration/active`
- `/northstar_0/calibration/capture_flag`

and the same pattern for `northstar_1` and `northstar_2`.

Use AdvantageScope, OutlineViewer, or another NT client to:

1. Set `active = true`
2. Show the included `charuco_board.png` to the camera from many distances and angles
3. Pulse `capture_flag = true` for each good frame you want saved
4. After collecting enough frames, set `active = false`

Northstar will write `calibration_new.yml` in its working directory and then exit. Rename that file to the camera-specific calibration filename expected by the launcher.

## macOS quick start

```bash
cd /path/to/RobotCode2026Public/northstar
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Then launch one process per camera:

```bash
./run-northstar-macos.sh /path/to/RobotCode2026Public/northstar
```

## Linux note

The upstream `requirements.txt` includes macOS-specific dependencies. The simplest first setup is macOS. If you use Linux, install the needed packages manually and expect to drop packages that fail for your platform.

## Robot-side switch

After Northstar is publishing frames, change:

`src/main/java/org/Griffins1884/frc2026/subsystems/vision/AprilTagVisionConstants.java`

from `VisionBackend.LIMELIGHT` to `VisionBackend.NORTHSTAR`.
