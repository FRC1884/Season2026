# FRC 1884 • 2026 REEFSCAPE Robot Code

> The official software stack for Team 1884 Griffins’ 2026 build season. Everything that hits the field—from drivetrain controls to the Reef Controls tablet—is developed and tested here.

## 2026 Season Highlights

- **Reef Controls v2** – A fully redesigned tablet web app with live counters, queue building, drag-and-drop reordering, and manual override for drivers and coaches.
- **Queue-aware autonomy** – Tablet selections become real commands through `TabletInterfaceTracker`, which feeds alignment goals straight into the swerve.
- **Drive + Turn SysId** – Separate characterization routines for the Kraken Pro drivetrain and steer stages so we can retune in the pit without touching code.
- **State-first subsystems** – Everything from the superstructure down uses AdvantageKit logging, hardware abstraction, and command-based “verbs” instead of long procedural scripts.
- **Simulation readiness** – maple-sim powered physics plus PathPlanner + Choreo let us vet autos and queue flows before the robot is wired.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/main/java/frc/robot` | Main robot project (subsystems, commands, configs). |
| `src/main/deploy/reefcontrols` | Tablet UI assets served by the robot (`index.html/css/js`). |
| `docs/` | Supplemental notes, coordinate frames, tuning logs. |
| `vendordeps/` | Vendor libraries pinned for 2026 (Phoenix 6, REVLib, etc.). |

## Getting Started

1. **Tools** – Install WPILib 2026 beta, JDK 17, and AdvantageScope.
2. **Clone** – `git clone https://github.com/frc1884/season2026.git`
3. **Build** – `./gradlew build` (desktop) or `./gradlew simulateJava` for sim run.
4. **Deploy** – `./gradlew deploy` (robot must be on the same network).
5. **Tablet UI** – With the robot running, browse to `http://<roboRIO>:5805` to launch Reef Controls v2.
6. **Remote tablets** – If you host the UI somewhere else (e.g., `reefcontrols.io` or a local file), add `?ntHost=<robot-ip>` (and optionally `&ntPort=<port>`) to the URL so the page knows which NT4 server to connect to.

## Reef Controls Templates

Reef Controls now renders from a year-specific JSON template so you can change layouts, buttons, assets, and queue text without touching `index.html`.

- Templates live in `src/main/deploy/reefcontrols/templates`. The file name (without `.json`) becomes the template id (e.g., `2025.json` → `?template=2025`).
- Point the tablet UI at a template with the `template` query param: `http://<roboRIO>:5805/?template=2025`. If omitted, the default template is used.
- Every template can override metadata (title/subtitle/favicon), section colors, reef node placements, control buttons/styles/actions, and queue copy.
- Use the visual builder at `docs/template-builder/index.html` to author templates. Open it locally in a browser, tweak metadata/counters/nodes/buttons, then click **Download JSON** and drop the file into the templates directory.
- Actions support topic deltas, toggles, literal publishes, and queue commands. If a topic field is left empty, the builder falls back to the stock Reef Controls topics.

After editing a template, redeploy (`./gradlew deploy`) or copy the JSON onto the RIO so the tablet can fetch the new layout.

## Tech Stack

**Credits** <br>
There are way too many people to thank and the list gets longer virtually every day, but we'd like to extend a massive thank you to every FRC team for publicizing their 2023-24 season repositories as inspiration for this project! Without your gracious professionalism, we would not have been able to get this massive project off the ground. Of course, we are also extremely grateful to team 6328 and everyone who has helped maintain the incredible framework that is AdvantageKit, as well as the extensive documentation that it thankfully has.

Here's a full list of the software libraries we are using, and the vendors accredited with them:
* Team 6328 Mechanical Advantage (AdvantageKit, AdvantageScope & URCL)
* SleipnirGroup (Choreo & ChoreoLib)
* Team 5516 Iron Maple (maple-sim)
* Team 3015 Ranger Robotics (PathPlanner & PathPlannerLib)
* CTR Electronics (Phoenix)
* PhotonVision (PhotonVision & PhotonLib)
* REV Robotics (REVlib)
* Studica (Studica NavX2)
* WPILib Suite (WPILib and the official FRC software ecosystem)

And to every team sharing 2024–2026 codebases—you continue to make FRC better. If you reuse anything here, let us know; we love contributing upstream.

*(Updated for the 2026 build season)*
