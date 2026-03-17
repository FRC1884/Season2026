# FRC 1884 • 2026 REBUILT Robot Code

> The official software stack for Team 1884 Griffins’ 2026 build season (REBUILT). Everything that hits the field, from drivetrain controls to the operator dashboard, lives here.

## 2026 Season Highlights

- **Operator Board UI** – Lightweight tablet web app (served by the robot) for state requests + at-a-glance telemetry.
- **Drive + Turn SysId** – Separate characterization routines for the Kraken Pro drivetrain and steer stages so we can retune in the pit without touching code.
- **State-first subsystems** – Everything from the superstructure down uses AdvantageKit logging, hardware abstraction, and command-based “verbs” instead of long procedural scripts.
- **Simulation readiness** – maple-sim powered physics lets us vet changes before the robot is wired.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/main/java/org/Griffins1884/frc2026` | Main robot project (subsystems, commands, configs). |
| `src/main/deploy/operatorboard` | Operator Board UI assets served by the robot (`index.html/css/js`). |
| `operatorboard-data` | Local persistent dashboard/runtime data mirrored with the roboRIO. |
| `docs/` | Supplemental notes, coordinate frames, tuning logs. |
| `vendordeps/` | Vendor libraries pinned for 2026 (Phoenix 6, REVLib, etc.). |

## Operational Checklists

- Match/practice verification checklist: `docs/ROBOT_FUNCTIONALITY_CHECKLIST.md`
- Includes a dedicated gyro failover + odometry-reset hardening validation block (Pigeon2 -> NavX fallback and vision suppression after resets)

## Getting Started

1. **Tools** – Install WPILib 2026 beta, JDK 17, and AdvantageScope.
2. **Clone** – `git clone https://github.com/frc1884/season2026.git`
3. **Build** – `./gradlew build` (desktop) or `./gradlew simulateJava` for sim run.
4. **Deploy** – `./gradlew deploy` (robot must be on the same network).
5. **Deploy + preserve saved data** – `./gradlew deployPreserve` to pull, merge, deploy, and re-push persistent dashboard data.
6. **Operator Board UI** – With the robot running, browse to `http://<roboRIO>:5805` to open the dashboard.
7. **Remote tablets** – If you host the UI elsewhere, add `?ntHost=<robot-ip>` (and optionally `&ntPort=<port>`) so the page knows which NT4 server to connect to.

## Persistent Data

- Local saved dashboard data lives in `operatorboard-data/`.
- roboRIO runtime saved data lives in `/home/lvuser/operatorboard-data`.
- Deploy-seeded defaults live in `src/main/deploy/operatorboard/default-data/`.
- PathPlanA autos are preserved across deploys through the `deployPreserve` workflow.
- The operator board now exposes a joystick-mapping tab, a subsystem/state description tab, and diagnostic bundle export endpoints.

See `docs/PERSISTENCE_ARCHITECTURE.md` for the full file inventory, sync rules, and conflict policy.

## Tech Stack

**Credits** <br>
There are way too many people to thank and the list gets longer virtually every day, but we'd like to extend a massive thank you to every FRC team for publicizing their 2023-24 season repositories as inspiration for this project! Without your gracious professionalism, we would not have been able to get this massive project off the ground. Of course, we are also extremely grateful to team 6328 and everyone who has helped maintain the incredible framework that is AdvantageKit, as well as the extensive documentation that it thankfully has.

Here's a full list of the software libraries we are using, and the vendors accredited with them:
* Team 6328 Mechanical Advantage (AdvantageKit, AdvantageScope & URCL)
* Team 5516 Iron Maple (maple-sim)
* CTR Electronics (Phoenix)
* PhotonVision (PhotonVision & PhotonLib)
* REV Robotics (REVlib)
* Studica (Studica NavX2)
* WPILib Suite (WPILib and the official FRC software ecosystem)

And to every team sharing 2024–2026 codebases—you continue to make FRC better. If you reuse anything here, let us know; we love contributing upstream.

*(Updated for the 2026 build season)*
