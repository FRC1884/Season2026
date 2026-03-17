package org.Griffins1884.frc2026;

import static org.Griffins1884.frc2026.Config.Controllers.getDriverController;
import static org.Griffins1884.frc2026.Config.Subsystems.AUTONOMOUS_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.DRIVETRAIN_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.TURRET_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.VISION_ENABLED;
import static org.Griffins1884.frc2026.GlobalConstants.MODE;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.BACK_LEFT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.BACK_RIGHT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.FRONT_LEFT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.FRONT_RIGHT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.GYRO_TYPE;
import static org.Griffins1884.frc2026.subsystems.vision.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import org.Griffins1884.frc2026.GlobalConstants.RobotMode;
import org.Griffins1884.frc2026.GlobalConstants.RobotType;
import org.Griffins1884.frc2026.OI.DriverMap;
import org.Griffins1884.frc2026.commands.DriveCommands;
import org.Griffins1884.frc2026.commands.TurretCommands;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardIOServer;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardTracker;
import org.Griffins1884.frc2026.subsystems.shooter.*;
import org.Griffins1884.frc2026.subsystems.swerve.*;
import org.Griffins1884.frc2026.subsystems.turret.TurretConstants;
import org.Griffins1884.frc2026.subsystems.turret.TurretIO;
import org.Griffins1884.frc2026.subsystems.turret.TurretIOKraken;
import org.Griffins1884.frc2026.subsystems.turret.TurretIOSim;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double ODOMETRY_RESET_VISION_SUPPRESS_SECONDS = 0.35;

  // Subsystems
  private final SwerveSubsystem drive;
  private SwerveDriveSimulation driveSimulation;
  private final TurretSubsystem turret;
  private final OperatorBoardTracker operatorBoard;

  // Controller
  private final DriverMap driver = getDriverController();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> characterizationChooser;
  private final Command characterizationIdleCommand;

  private final Superstructure superstructure;
  private final Vision vision;
  private boolean autoAllianceZeroed = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Validate the declarative mechanism catalog up front so config errors fail early.
    RobotMechanismDefinitions.all();
    characterizationChooser = new LoggedDashboardChooser<>("Characterization/Diagnostics");
    characterizationIdleCommand = Commands.none();
    characterizationChooser.addDefaultOption("None", characterizationIdleCommand);

    if (DRIVETRAIN_ENABLED) {
      drive =
          switch (MODE) {
            case REAL:
              // Real robot, instantiate hardware IO implementations
              yield new SwerveSubsystem(
                  switch (GYRO_TYPE) {
                    case PIGEON -> new GyroIOPigeon2();
                    case NAVX -> new GyroIONavX();
                    case ADIS -> new GyroIO() {};
                  },
                  new ModuleIOFullKraken(FRONT_LEFT),
                  new ModuleIOFullKraken(FRONT_RIGHT),
                  new ModuleIOFullKraken(BACK_LEFT),
                  new ModuleIOFullKraken(BACK_RIGHT));
            case SIM:
              // Create a maple-sim swerve drive simulation instance
              this.driveSimulation =
                  new SwerveDriveSimulation(
                      SwerveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
              // Add the simulated drivetrain to the simulation field
              SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

              // Sim robot, instantiate physics sim IO implementations
              yield new SwerveSubsystem(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOSim(driveSimulation.getModules()[0]),
                  new ModuleIOSim(driveSimulation.getModules()[1]),
                  new ModuleIOSim(driveSimulation.getModules()[2]),
                  new ModuleIOSim(driveSimulation.getModules()[3]));

            default:
              // Replayed robot, disable IO implementations
              yield new SwerveSubsystem(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
          };
      superstructure = new Superstructure(drive);

    } else {
      drive = null;
      superstructure = new Superstructure(null);
    }

    if (TURRET_ENABLED) {
      turret =
          switch (MODE) {
            case REAL -> new TurretSubsystem(new TurretIOKraken());
            case SIM -> new TurretSubsystem(new TurretIOSim());
            default -> new TurretSubsystem(new TurretIO() {});
          };

      superstructure.setTurret(turret);
    } else {
      turret = null;
    }

    if (MODE == RobotMode.SIM && turret != null && drive != null) {
      turret.setDefaultCommand(
          TurretCommands.autoAimWhileMovingToTarget(
              turret,
              drive::getPose,
              pose -> Optional.of(TurretConstants.getSimTarget()),
              drive::getFieldVelocity,
              drive::getFieldAcceleration));
      superstructure.setTurretExternalControl(true);
    }

    if (VISION_ENABLED) {
      vision =
          switch (MODE) {
            case REAL, SIM ->
                new Vision(
                    drive,
                    drive::getPose,
                    () -> Math.toRadians(drive.getYawRateDegreesPerSec()),
                    LEFT_CAM_ENABLED
                        ? new AprilTagVisionIOLimelight(LEFT_CAM_CONSTANTS, drive)
                        : new VisionIO() {},
                    RIGHT_CAM_ENABLED
                        ? new AprilTagVisionIOLimelight(RIGHT_CAM_CONSTANTS, drive)
                        : new VisionIO() {},
                    MIDDLE_RIGHT_CAM_ENABLED
                        ? new AprilTagVisionIOLimelight(MIDDLE_RIGHT_CAM_CONSTANTS, drive)
                        : new VisionIO() {});
            default -> new Vision(drive, new VisionIO() {}, new VisionIO() {});
          };
    } else vision = null;

    if (Config.Subsystems.WEBUI_ENABLED) {
      operatorBoard =
          new OperatorBoardTracker(
              new OperatorBoardIOServer(), superstructure, drive, turret, vision);
    } else {
      operatorBoard = null;
    }

    if (drive != null) {
      drive.setOdometryResetListener(this::handleOdometryReset);
    }

    if (DRIVETRAIN_ENABLED && drive != null) {
      characterizationChooser.addOption(
          "Drive | SysId (Full Routine)", drive.sysIdRoutine().ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(drive).ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | Feedforward Characterization",
          DriveCommands.feedforwardCharacterization(drive).ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | SysId (Dynamic Forward)",
          drive.sysIdDynamic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Drive | SysId (Dynamic Reverse)",
          drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turn | SysId (Full Routine)", drive.sysIdTurnRoutine().ignoringDisable(true));
      characterizationChooser.addOption(
          "Turn | SysId (Quasistatic Forward)",
          drive.sysIdTurnQuasistatic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turn | SysId (Quasistatic Reverse)",
          drive.sysIdTurnQuasistatic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turn | SysId (Dynamic Forward)",
          drive.sysIdTurnDynamic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turn | SysId (Dynamic Reverse)",
          drive.sysIdTurnDynamic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
    }

    SmartDashboard.putBoolean("drive/test", DriveCommands.getTest().get());

    superstructure.registerSuperstructureCharacterization(() -> characterizationChooser);
    if (turret != null) {
      Command turretSysIdFull =
          Commands.sequence(
                  turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                  Commands.waitSeconds(0.5),
                  turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                  Commands.waitSeconds(0.5),
                  turret.sysIdDynamic(SysIdRoutine.Direction.kForward),
                  Commands.waitSeconds(0.5),
                  turret.sysIdDynamic(SysIdRoutine.Direction.kReverse))
              .ignoringDisable(true);
      characterizationChooser.addOption("Turret | SysId (Full Routine)", turretSysIdFull);
      characterizationChooser.addOption(
          "Turret | SysId (Quasistatic Forward)",
          turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turret | SysId (Quasistatic Reverse)",
          turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turret | SysId (Dynamic Forward)",
          turret.sysIdDynamic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
      characterizationChooser.addOption(
          "Turret | SysId (Dynamic Reverse)",
          turret.sysIdDynamic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
    }

    // Configure the button bindings
    configureDriverButtonBindings();

    superstructure.setAutoStartPoseSupplier(
        operatorBoard != null ? operatorBoard::getQueuedStartPose : Optional::empty);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverButtonBindings() {
    if (DRIVETRAIN_ENABLED) {
      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDriveCommand(
              drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()));

      // Held override: robot-relative drive with the robot front/back and rotation flipped.
      driver
          .alignWithBall()
          .whileTrue(
              DriveCommands.joystickDriveRobotRelativeFlippedCommand(
                  drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()));

      // // Reset gyro to 0° when B button is pressed
      driver
          .resetOdometry()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        if (drive == null) {
                          return;
                        }
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isEmpty()) {
                          Logger.recordOutput("Odometry/AllianceZero/Failed", true);
                          Logger.recordOutput("Odometry/AllianceZero/Reason", "ALLIANCE_UNKNOWN");
                          return;
                        }
                        drive.zeroGyroAndOdometryToAllianceWall(alliance.get());
                      },
                      drive)
                  .ignoringDisable(true));
    }
    driver
        .shootToggle()
        .onTrue(Commands.runOnce(() -> superstructure.setShootEnabled(true)))
        .onFalse(Commands.runOnce(() -> superstructure.setShootEnabled(false)));
    if (GlobalConstants.ROBOT != RobotType.DBOT) {
      driver
          .intakeRollersHold()
          .onTrue(Commands.runOnce(() -> superstructure.setIntakeRollersHeld(true)))
          .onFalse(Commands.runOnce(() -> superstructure.setIntakeRollersHeld(false)));
      driver.intakeDeployToggle().onTrue(Commands.runOnce(superstructure::toggleIntakeDeploy));
    } else {
      driver
          .intakeDeployToggle()
          .onTrue(Commands.runOnce(() -> superstructure.setIntakeDeployed(true)))
          .onFalse(Commands.runOnce(() -> superstructure.setIntakeDeployed(false)));
    }
  }

  /**
   * Use this to pass the autonomwous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous, or null if the auto chooser is not initialized.
   */
  public Command getAutonomousCommand() {
    if (!AUTONOMOUS_ENABLED) return null;
    Command selected = operatorBoard != null ? operatorBoard.getAutonomousCommand() : null;
    superstructure.setAutonomousHoldEnabled(selected == null);
    return selected;
  }

  public Command getCharacterizationCommand() {
    Command selected = characterizationChooser.get();
    return selected == characterizationIdleCommand ? null : selected;
  }

  public Command getDriveSysIdCommand() {
    if (!DRIVETRAIN_ENABLED || drive == null) {
      return Commands.none();
    }
    return drive.sysIdRoutine().ignoringDisable(true);
  }

  public void resetSimulationField() {
    if (MODE != RobotMode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /** Auto-zero gyro/odometry once when disabled and alliance is known. */
  public void tryAutoZeroOdometryToAllianceWall() {
    if (autoAllianceZeroed || drive == null) {
      return;
    }
    if (!DriverStation.isDisabled()) {
      return;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;
    }
    drive.zeroGyroAndOdometryToAllianceWall(alliance.get());
    autoAllianceZeroed = true;
    Logger.recordOutput("Odometry/AutoAllianceZeroed", true);
  }

  private void handleOdometryReset() {
    if (vision == null) {
      return;
    }
    vision.resetPoseHistory();
    vision.suppressVisionForSeconds(ODOMETRY_RESET_VISION_SUPPRESS_SECONDS);
  }

  public void displaySimFieldToAdvantageScope() {
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    if (turret != null) {
      Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
      Translation2d turretTranslation =
          robotPose
              .getTranslation()
              .plus(TurretConstants.MOUNT_OFFSET_METERS.rotateBy(robotPose.getRotation()));
      Rotation2d turretRotation =
          robotPose.getRotation().plus(Rotation2d.fromRadians(turret.getPositionRad()));
      Translation2d turretTarget =
          MODE == RobotMode.SIM
              ? TurretCommands.predictShootingWhileMoving(
                  drive::getPose,
                  TurretConstants::getSimTarget,
                  drive::getFieldVelocity,
                  drive::getFieldAcceleration)
              : TurretConstants.getSimTarget();
      Logger.recordOutput(
          "FieldSimulation/TurretPose", new Pose2d(turretTranslation, turretRotation));
      Logger.recordOutput(
          "FieldSimulation/TurretTarget", new Pose2d(turretTarget, new Rotation2d()));
    }
  }

  public void setTeleopState() {
    superstructure.setAutonomousHoldEnabled(false);
    superstructure.setAutoStateEnabled(true);
  }
}
