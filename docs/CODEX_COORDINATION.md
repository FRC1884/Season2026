# Codex Coordination

## 2026-03-17T00:00:00Z

- Agent: GPT-5 Codex session working on persistence, deploy-sync, joystick mapping, subsystem/state metadata, and diagnostic bundle export.
- Intended shared-file touch points:
  - `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardTracker.java`
  - `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardIOServer.java`
  - `src/main/deploy/operatorboard/index.html`
  - `src/main/deploy/operatorboard/index.css`
  - `src/main/deploy/operatorboard/index.js`
  - `build.gradle`
  - `README.md`
- Strategy:
  - Keep most implementation in new files under dedicated persistence/diagnostic packages.
  - Re-check git status before editing any shared operator-board asset.
  - Avoid reverting or reshaping unrelated in-flight changes from other agents.

## 2026-03-17T00:30:00Z

- Shared files actually touched in this pass:
  - `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardTracker.java`
  - `src/main/deploy/operatorboard/index.html`
  - `src/main/deploy/operatorboard/index.css`
  - `src/main/deploy/operatorboard/index.js`
  - `build.gradle`
  - `README.md`
- New implementation files were added for persistence, diagnostic bundles, deploy-preserve scripts, and default seeded JSON assets.

## 2026-03-17T12:05:00Z

- Agent: GPT-5 Codex session starting ballistics groundwork for configurable shot geometry and offline sweep export.
- Intended shared-file touch points:
  - `build.gradle`
- Strategy:
  - Avoid changing live shooter/turret behavior in this pass.
  - Add new ballistics model/export files under a dedicated package.
  - Use a standalone Gradle task for offline export so other in-flight robot changes are unaffected.

## 2026-03-17T14:20:00Z

- Shared files actually touched in this pass:
  - `build.gradle`
- New files added:
  - `src/main/java/org/Griffins1884/frc2026/util/ballistics/ShotModel.java`
  - `src/main/java/org/Griffins1884/frc2026/util/ballistics/ShotModelConfig.java`
  - `src/main/java/org/Griffins1884/frc2026/util/ballistics/AdvancedBallisticsShotModel.java`
  - `src/main/java/org/Griffins1884/frc2026/util/ballistics/ShotSweepExporterMain.java`
  - `src/test/java/org/Griffins1884/frc2026/util/ballistics/AdvancedBallisticsShotModelTest.java`
- Notes for other agents:
  - Live turret/shooter command paths were intentionally left unchanged.
  - Offline export currently writes to `build/reports/ballistics/`.
  - Unrelated `src/main/deploy/pathplana/autos/*` changes were observed and intentionally left alone.

## 2026-03-28T00:00:00Z

- Agent: GPT-5 Codex session reworking operator-board controller profiles, preview-driven joystick mapping, and runtime driver-profile consumption.
- Shared files touched in this pass:
  - `src/main/deploy/operatorboard/index.html`
  - `src/main/deploy/operatorboard/index.css`
  - `src/main/deploy/operatorboard/index.js`
  - `src/main/deploy/operatorboard/default-data/joystick-mappings.json`
  - `operatorboard-data/joystick-mappings.json`
  - `src/main/java/org/Griffins1884/frc2026/Config.java`
  - `src/main/java/org/Griffins1884/frc2026/OI/*`
  - `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardDataModels.java`
- Notes for the parallel NamedCommands work:
  - The operator board now treats these saved target IDs as the authoritative button-action identifiers:
    - `action:alignWithBall`
    - `action:shootToggle`
    - `action:intakeRollersHold`
    - `action:intakeDeployToggle`
    - `action:resetOdometry`
  - Please expose matching NamedCommands or an equivalent command registry for those IDs, then we can remove the remaining action-specific glue in `RobotContainer.configureDriverButtonBindings()`.
  - Continuous drive axes are now identified separately as `drive.strafe`, `drive.forward`, and `drive.rotate`.

## 2026-03-29T00:00:00Z

- Branch naming policy for Codex and other AI-assisted sessions:
  - Do not create or push Git branches whose names include `codex`, `ai`, or other AI signifiers.
  - Use neutral descriptive branch names instead, for example feature- or fix-oriented names without agent branding.
  - If an AI-generated branch with those signifiers already exists on GitHub, delete it after moving the work to a neutral branch.
