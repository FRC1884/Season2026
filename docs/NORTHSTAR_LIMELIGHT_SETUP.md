# Northstar With Existing Limelights

## Bottom line

Northstar is not a direct LLOS replacement. It expects to run as a separate Python process, publish its own NetworkTables schema, and read frames through OpenCV/Pylon/GStreamer-style capture backends.

The workable path with your current hardware is:

1. Keep each Limelight as an IP camera source.
2. Run one Northstar process per camera on an external Linux/macOS coprocessor.
3. Point each Northstar instance at the Limelight MJPEG stream URL.
4. Switch the robot AprilTag backend from `LIMELIGHT` to `NORTHSTAR`.

## Robot-side support added in this repo

- `AprilTagVisionIONorthstar` subscribes to `northstar_X/output/observations`
- `RobotContainer` can now instantiate Northstar-backed AprilTag cameras
- `AprilTagVisionConstants` now has:
  - `BACKEND`
  - `LEFT_CAM_NORTHSTAR_CONFIG`
  - `RIGHT_CAM_NORTHSTAR_CONFIG`
  - `MIDDLE_RIGHT_CAM_NORTHSTAR_CONFIG`

The default is still `LIMELIGHT`, so nothing changes until you flip it.

## How to run it

### 1. On the Northstar host

Clone `RobotCode2026Public`, then run one process per camera with unique `device_id` values:

- `northstar_0`
- `northstar_1`
- `northstar_2`

Each local Northstar `config.json` should point back to the roboRIO NT server and leave capture on the default OpenCV path:

```json
{
  "device_id": "northstar_0",
  "server_ip": "10.xx.yy.2",
  "apriltags_stream_port": 8000,
  "objdetect_stream_port": 8001,
  "capture_impl": "",
  "obj_detect_model": "",
  "obj_detect_max_fps": -1,
  "apriltags_enable": true,
  "objdetect_enable": false,
  "video_folder": ""
}
```

Use one config file and one calibration file per camera.

### 2. Use Limelight stream URLs as the Northstar camera IDs

This repo now publishes camera IDs like:

- `http://limelight-left.local:5800/stream.mjpg`
- `http://limelight-right.local:5800/stream.mjpg`
- `http://limelight-side.local:5800/stream.mjpg`

Those are the values Northstar receives through `northstar_X/config/camera_id`.

If mDNS is unreliable, replace them with fixed IPs.

### 3. Calibrate each Limelight stream for Northstar

Northstar still needs standard camera intrinsics. Do not reuse Limelight MegaTag tuning assumptions as calibration.

You need one Northstar calibration file per Limelight/lens:

- `calibration-left.yml`
- `calibration-right.yml`
- `calibration-side.yml`

### 4. Switch the robot backend

In [AprilTagVisionConstants.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/subsystems/vision/AprilTagVisionConstants.java), change:

```java
public static final VisionBackend BACKEND = VisionBackend.LIMELIGHT;
```

to:

```java
public static final VisionBackend BACKEND = VisionBackend.NORTHSTAR;
```

## What this does not replace yet

- [AutoAlignToFuelCommand.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/commands/AutoAlignToFuelCommand.java)
- [GamePieceVisionIOLimelight.java](/Users/jonathanst-georges/Documents/season2026/src/main/java/org/Griffins1884/frc2026/subsystems/vision/GamePieceVisionIOLimelight.java)

Those still assume Limelight-specific `tx`/`ty` tables. The new path only covers AprilTag odometry right now.
