External Web UI (NetworkTables-Only)

Goal
- Run the UI on an iPad (or any device) without hosting HTTP on the roboRIO.
- Drive all controls/state via NetworkTables (NT4), using the robot as the NT server only.

Topics (table: `CoralRegistry`)
- Outputs (Robot -> UI)
  - `CoralRegistry/AutonomousEnabled` (boolean)
  - `CoralRegistry/TargetFace` (string: `REEF_1`..`REEF_6`)
  - `CoralRegistry/TargetSide` (string: `LEFT` or `RIGHT`)
  - `CoralRegistry/TargetLevel` (string: `L1`..`L4`)
  - `CoralRegistry/TargetName` (string: path/label, e.g. `Reef_3_L_Blue`)
  - `CoralRegistry/Branch/<branch>` (boolean: true when DONE), where `<branch>` is one of:
    `F1_A`, `F1_B`, ..., `F6_A`, `F6_B`.

- Controls (UI -> Robot) under `CoralRegistry/Control/*`
  - `Control/AutonomousEnabled` (boolean): enable/disable autonomy coordinator
  - `Control/TargetFace` (string: `REEF_1`..`REEF_6`)
  - `Control/TargetSide` (string: `LEFT` or `RIGHT`)
  - `Control/TargetLevel` (string: `L1`..`L4`)
  - `Control/TargetName` (string): optional direct target label
  - `Control/TriggerLoad` (boolean): set to true momentarily to simulate a coral load event
  - `Control/TriggerRelease` (boolean): set to true momentarily to simulate a coral release
  - `Control/Send` (boolean): momentary; asks the system to act on current selection

Notes
- `TriggerLoad`, `TriggerRelease`, and `Send` are edge-triggered; set true then false.
- All state also mirrors into AdvantageKit logs under `Log/CoralRegistry/*`.
- The embedded HTTP server is gated by `Config.WebUIConfig.ENABLED` and is false by default; the robot does not host any web pages.

Connecting from a Web App
- Use an NT4 WebSocket client and connect to `ws://<roborio-host-or-ip>:5810/nt/` (default WPILib NT4).
- On the iPad, a PWA or a static site (hosted anywhere else) can connect to the robot’s NT server over the field network.
- Populate the control topics listed above to drive autonomy and read back robot state from the output topics.

Tips
- mDNS host: `roborio-<TEAM>-frc.local`. Example: `roborio-32132-frc.local`.
- For dashboards: AdvantageScope or Glass can also view these topics to validate wiring.
