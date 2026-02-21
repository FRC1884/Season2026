# Architecture Map

## Entry Points
- `src/main/java/frc/robot/Main.java`: `RobotBase.startRobot(Robot::new)`.
- `src/main/java/frc/robot/Robot.java`: extends `LoggedRobot`, initializes AdvantageKit logging,
  constructs `RobotContainer`, runs `CommandScheduler` in `robotPeriodic()`, and handles auto/test/sim.

## Container / Wiring
- `src/main/java/frc/robot/RobotContainer.java`:
  - Constructs subsystems based on `Config.Subsystems` and `GlobalConstants.MODE`.
  - Creates `SwerveSubsystem` with real/sim/replay IO implementations.
  - Creates `Vision` and passes `SwerveSubsystem` as the `VisionConsumer`.
  - Creates `Superstructure` with a `drive::getPose` supplier.
  - Optionally creates `TabletInterfaceTracker` for the web UI.
  - Sets up `LoggedDashboardChooser` for auto and characterization.

## Subsystem + IO Pattern
- IO abstraction with AdvantageKit AutoLog inputs:
  - `swerve.subsystems.org.Griffins1884.frc2026.GyroIO`, `ModuleIO` with `*InputsAutoLogged`.
  - `arms.generic.org.Griffins1884.frc2026.GenericArmSystemIO`,
    `elevators.generic.org.Griffins1884.frc2026.GenericElevatorSystemIO`,
    `rollers.generic.org.Griffins1884.frc2026.GenericRollerSystemIO`,
    `turrets.generic.org.Griffins1884.frc2026.GenericTurretSystemIO`.
- Hardware ownership lives in IO implementations:
  - Swerve: `GyroIOPigeon2`, `GyroIONavX`, `ModuleIOSpark`, `ModuleIOHalfSpark`,
    `ModuleIOFullKraken`, `ModuleIOSim`.
  - Turret: `TurretIOSparkMax`, `TurretIOSparkFlex`, `TurretIOKraken`, `TurretIOSim`.
  - Arm/roller/elevator subsystems:
    `PivotIOFlex`/`PivotIOMax`/`PivotIOSim`,
    `ShooterIOFlex`/`ShooterIOMax`/`ShooterIOSim`,
    `IntakeIOFlex`/`IntakeIOMax`/`IntakeIOSim`,
    `LEDIOPWM`/`LEDIOSim`.
  - Template implementations live under `src/main/java/frc/robot/subsystems/exampleClasses`.
- Subsystems extend `SubsystemBase` or generic base classes:
  - `GenericPositionArmSystem`, `GenericPositionElevatorSystem`,
    `GenericVelocityRollerSystem`, `GenericVoltageRollerSystem`,
    `GenericPositionTurretSystem`.

## Commands & Bindings
- Command factories live in `src/main/java/frc/robot/commands`
  (e.g., `DriveCommands`, `AutoAlignToPoseCommand`, `AutoCommands`).
- Button bindings are in `RobotContainer.configureDriverButtonBindings()`.
  Operator bindings are in `RobotContainer.configureOperatorButtonBindings()`
  (turret zero/preset/manual/auto-aim controls when enabled).
- Default swerve command is set in `RobotContainer` using `DriveCommands.joystickDrive(...)`.

## Constants
- Global/config: `src/main/java/frc/robot/GlobalConstants.java`, `src/main/java/frc/robot/Config.java`.
- Subsystem constants: `frc.robot.subsystems.*.*Constants`
  (e.g., `SwerveConstants`, `PivotConstants`, `ShooterConstants`, `LEDConstants`).
- Alignment constants: `src/main/java/frc/robot/commands/AlignConstants.java`.

## Superstructure / State Machine
- `src/main/java/frc/robot/subsystems/Superstructure.java`: wraps `Rollers`, `Elevators`, `Arms`;
  currently minimal periodic logic.
- `src/main/java/frc/robot/StateMachine.java` with
  `util.org.Griffins1884.frc2026.StateGraph` and `util.org.Griffins1884.frc2026.Transition` provides a state-machine framework,
  but there are no current subclasses.

## Loop Scheduling / Timing
- WPILib command-based scheduling in `Robot.robotPeriodic()` via `CommandScheduler`.
- High-rate odometry threads:
  - `src/main/java/frc/robot/subsystems/swerve/PhoenixOdometryThread.java`
  - `src/main/java/frc/robot/subsystems/swerve/SparkOdometryThread.java`
- Simulation loop: `Robot.simulationPeriodic()` calls `SimulatedArena`.

## Dependency Graph (Packages + Key Edges)
```
org.Griffins1884.frc2026.Main
  -> org.Griffins1884.frc2026.Robot
     -> org.Griffins1884.frc2026.RobotContainer
        -> swerve.subsystems.org.Griffins1884.frc2026.SwerveSubsystem
            -> swerve.subsystems.org.Griffins1884.frc2026.Module (owns ModuleIO*)
            -> swerve.subsystems.org.Griffins1884.frc2026.GyroIO*
        -> turret.subsystems.org.Griffins1884.frc2026.TurretSubsystem
        -> vision.subsystems.org.Griffins1884.frc2026.Vision (owns VisionIO*)
            -> VisionConsumer: SwerveSubsystem.accept(...)
        -> subsystems.org.Griffins1884.frc2026.Superstructure
            -> rollers.generic.org.Griffins1884.frc2026.Rollers
                -> intake.subsystems.org.Griffins1884.frc2026.IntakeSubsystem
                -> shooter.subsystems.org.Griffins1884.frc2026.ShooterSubsystem
            -> elevators.generic.org.Griffins1884.frc2026.Elevators
            -> arms.generic.org.Griffins1884.frc2026.Arms
                -> frc.robot.subsystems.pivot.PivotSubsystem
        -> objectivetracker.subsystems.org.Griffins1884.frc2026.TabletInterfaceTracker (optional)
        -> OI.org.Griffins1884.frc2026.DriverMap / OperatorMap
frc.robot.commands.* -> depends on subsystems + GlobalConstants + frc.robot.util.*
frc.robot.util.* -> shared math/utilities used across commands/subsystems
```

## Sensor and State Flow
- Swerve:
  - `GyroIO.updateInputs` + `ModuleIO.updateInputs` -> `SwerveSubsystem.periodic()`.
  - Inputs drive odometry (`SwerveDrivePoseEstimator`) and are logged via AdvantageKit.
  - `DriveCommands` reads pose/rotation to compute chassis speeds and alignment.
- Vision:
  - `VisionIO.updateInputs` -> `Vision.periodic()`.
  - Accepted observations are passed to `SwerveSubsystem.accept(...)` to update pose estimation.
- Other subsystems:
  - IO `updateInputs` -> subsystem periodic control loop -> `setVoltage`/`runVolts` output.
  - Telemetry recorded via `Logger.processInputs` and `Logger.recordOutput`.
- Turret:
  - `TurretIO.updateInputs` -> `TurretSubsystem.periodic()` control loop.
  - Setpoints and soft limits enforced in software; telemetry logged under `Turret/*`.
  - Auto-aim uses `Vision.getBestTargetTranslation(...)` + `SwerveSubsystem.getPose()` and
    `TurretUtil.turretAngleToTarget(...)` when bound.
