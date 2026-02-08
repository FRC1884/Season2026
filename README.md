# FRC 1884 • 2026 REBUILT Robot Code

> The official software stack for Team 1884 Griffins’ 2026 build season (REBUILT). Everything that hits the field, from drivetrain controls to the operator dashboard, lives here.

## 2026 Season Highlights

- **Operator Board UI** – Lightweight tablet web app (served by the robot) for state requests + at-a-glance telemetry.
- **Drive + Turn SysId** – Separate characterization routines for the Kraken Pro drivetrain and steer stages so we can retune in the pit without touching code.
- **State-first subsystems** – Everything from the superstructure down uses AdvantageKit logging, hardware abstraction, and command-based “verbs” instead of long procedural scripts.
- **Simulation readiness** – maple-sim powered physics plus PathPlanner let us vet changes before the robot is wired.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/main/java/org/Griffins1884/frc2026` | Main robot project (subsystems, commands, configs). |
| `src/main/deploy/operatorboard` | Operator Board UI assets served by the robot (`index.html/css/js`). |
| `docs/` | Supplemental notes, coordinate frames, tuning logs. |
| `vendordeps/` | Vendor libraries pinned for 2026 (Phoenix 6, REVLib, etc.). |

## Getting Started

1. **Tools** – Install WPILib 2026 beta, JDK 17, and AdvantageScope.
2. **Clone** – `git clone https://github.com/frc1884/season2026.git`
3. **Build** – `./gradlew build` (desktop) or `./gradlew simulateJava` for sim run.
4. **Deploy** – `./gradlew deploy` (robot must be on the same network).
5. **Operator Board UI** – With the robot running, browse to `http://<roboRIO>:5805` to open the dashboard.
6. **Remote tablets** – If you host the UI elsewhere, add `?ntHost=<robot-ip>` (and optionally `&ntPort=<port>`) so the page knows which NT4 server to connect to.

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
