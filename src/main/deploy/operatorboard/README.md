# Operator Board UI

This folder contains the robot-served dashboard assets for the REBUILT operator workflow.

## Pages

### `index.html`

The main operator board. It is split into:

- `Teleop`: superstructure state requests, driver indicators, match telemetry, diagnostics, and maintenance controls.
- `Auto`: deploy-backed auto preview and selection, field view, and selected-auto status.

The page talks to the robot over NT4 and uses the `/OperatorBoard/v1` topic contract.

### `runtime-config.html`

The runtime profile editor. This page changes the live runtime logging/tuning profile without a code redeploy.

It controls:

- `loggingMode`
  - `COMP`: compact default telemetry set for normal robot use.
  - `DEBUG`: expanded default telemetry set.
- `tuningEnabled`
  - Allows runtime tunable inputs to stay editable.
- `debugSubsystems`
  - In `COMP`, checked subsystems inherit the expanded debug signal set.
  - In `DEBUG`, every subsystem is already treated as debug-enabled.
- `loggedSignals`
  - Explicit mechanism telemetry groups to log.
- `publishedSignals`
  - Explicit mechanism telemetry groups to publish live to NetworkTables.

Important behavior:

- Empty `loggedSignals` and `publishedSignals` lists do not mean "log/publish nothing".
- The robot falls back to mode defaults.
  - `COMP` default signals: `IDENTITY`, `CONNECTION`, `FAULTS`, `HEALTH`, `TARGET`
  - `DEBUG` default signals: every mechanism signal

## Local Presets

Runtime config presets are stored in browser local storage on the current device only.

- Saving a preset does not send anything to the robot.
- Loading a preset only fills out the form.
- Press `Apply To Robot` to publish the current JSON to the live runtime manager.

## Robot Side Wiring

The runtime config page publishes to:

- `/OperatorBoard/v1/ToRobot/RuntimeProfileSpec`
- `/OperatorBoard/v1/ToRobot/ApplyRuntimeProfile`
- `/OperatorBoard/v1/ToRobot/ResetRuntimeProfile`

It reads back:

- `/OperatorBoard/v1/ToDashboard/RuntimeProfileState`
- `/OperatorBoard/v1/ToDashboard/RuntimeProfileStatus`

The auto tab reads deployed PathPlanA autos from `/planner-autos/index.json` and publishes a single auto-selection topic:

- `/OperatorBoard/v1/ToRobot/SelectedAutoId`

The robot then loads the selected deploy auto into the existing auto-align execution path.

## Useful URLs

- Main board: `http://<roborio>:5805/index.html`
- Runtime config: `http://<roborio>:5805/runtime-config.html`

If the page is hosted somewhere other than the robot web server, add query params:

- `?ntHost=<robot-ip>`
- `&ntPort=<port>` if you are not using the default NT4 port
