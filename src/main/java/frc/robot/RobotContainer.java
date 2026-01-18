// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Config.Controllers.getDriverController;
import static frc.robot.Config.Controllers.getOperatorController;
import static frc.robot.Config.Subsystems.AUTONOMOUS_ENABLED;
import static frc.robot.Config.Subsystems.DRIVETRAIN_ENABLED;
import static frc.robot.Config.Subsystems.TURRET_ENABLED;
import static frc.robot.Config.Subsystems.VISION_ENABLED;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.GlobalConstants.robotSwerveMotors;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.GYRO_TYPE;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkFlex;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.*;
import java.util.Optional;
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
  // Subsystems
  private final SwerveSubsystem drive;
  private SwerveDriveSimulation driveSimulation;
  private final TurretSubsystem turret;

  // Controller
  private final DriverMap driver = getDriverController();

  private final OperatorMap operator = getOperatorController();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> characterizationChooser;
  private final Command autoIdleCommand;
  private final Command characterizationIdleCommand;

  private final Superstructure superstructure;
  private final VisionTargetProvider vision;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
                  switch (robotSwerveMotors) {
                    case FULLSPARK -> new ModuleIOSpark(FRONT_LEFT);
                    case HALFSPARK -> new ModuleIOHalfSpark(FRONT_LEFT);
                    case FULLKRACKENS -> new ModuleIOFullKraken(FRONT_LEFT);
                  },
                  switch (robotSwerveMotors) {
                    case FULLSPARK -> new ModuleIOSpark(FRONT_RIGHT);
                    case HALFSPARK -> new ModuleIOHalfSpark(FRONT_RIGHT);
                    case FULLKRACKENS -> new ModuleIOFullKraken(FRONT_RIGHT);
                  },
                  switch (robotSwerveMotors) {
                    case FULLSPARK -> new ModuleIOSpark(BACK_LEFT);
                    case HALFSPARK -> new ModuleIOHalfSpark(BACK_LEFT);
                    case FULLKRACKENS -> new ModuleIOFullKraken(BACK_LEFT);
                  },
                  switch (robotSwerveMotors) {
                    case FULLSPARK -> new ModuleIOSpark(BACK_RIGHT);
                    case HALFSPARK -> new ModuleIOHalfSpark(BACK_RIGHT);
                    case FULLKRACKENS -> new ModuleIOFullKraken(BACK_RIGHT);
                  });
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
      superstructure = new Superstructure(() -> drive.getPose());

      autoIdleCommand = Commands.none();
      if (AUTONOMOUS_ENABLED) {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        autoChooser.addDefaultOption("Do Nothing", autoIdleCommand);
      } else {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser.addDefaultOption("Do Nothing", autoIdleCommand);
      }

    } else {
      drive = null;
      autoChooser = null;
      autoIdleCommand = null;
      superstructure = new Superstructure(null);
    }

    if (TURRET_ENABLED) {
      turret =
          switch (MODE) {
            case REAL ->
                new TurretSubsystem(
                    switch (TurretConstants.MOTOR_CONTROLLER) {
                      case SPARK_MAX -> new TurretIOSparkMax();
                      case SPARK_FLEX -> new TurretIOSparkFlex();
                      case KRAKEN_X60, KRAKEN_X40 -> new TurretIOKraken();
                    });
            case SIM -> new TurretSubsystem(new TurretIOSim());
            default -> new TurretSubsystem(new TurretIO() {});
          };
    } else {
      turret = null;
    }
    if (superstructure != null) {
      superstructure.setTurret(turret);
    }
    if (MODE == RobotMode.SIM && turret != null && drive != null) {
      turret.setDefaultCommand(
          TurretCommands.autoAimToTarget(
              turret, drive::getPose, pose -> Optional.of(TurretConstants.getSimTarget())));
      if (superstructure != null) {
        superstructure.setTurretExternalControl(true);
      }
    }

    if (AUTONOMOUS_ENABLED) {
      if (drive != null) {
        AutoCommands.registerAutoCommands(superstructure, drive);
      }
    }

    if (VISION_ENABLED) {
      if (IS_LIMELIGHT) {
        vision =
            switch (MODE) {
              case REAL ->
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
                      BACK_CAM_ENABLED
                          ? new AprilTagVisionIOLimelight(BACK_CAM_CONSTANTS, drive)
                          : new VisionIO() {});
              case SIM ->
                  new Vision(
                      drive,
                      LEFT_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              LEFT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {},
                      RIGHT_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              RIGHT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {},
                      BACK_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              BACK_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {});
              default -> new Vision(drive, new VisionIO() {}, new VisionIO() {});
            };
      } else {
        vision =
            switch (MODE) {
              case REAL ->
                  new Vision(
                      drive,
                      LEFT_CAM_ENABLED
                          ? new AprilTagVisionIOPhotonVision(LEFT_CAM_CONSTANTS)
                          : new VisionIO() {},
                      RIGHT_CAM_ENABLED
                          ? new AprilTagVisionIOPhotonVision(RIGHT_CAM_CONSTANTS)
                          : new VisionIO() {},
                      BACK_CAM_ENABLED
                          ? new AprilTagVisionIOPhotonVision(BACK_CAM_CONSTANTS)
                          : new VisionIO() {});
              case SIM ->
                  new Vision(
                      drive,
                      LEFT_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              LEFT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {},
                      RIGHT_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              RIGHT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {},
                      BACK_CAM_ENABLED
                          ? new VisionIOPhotonVisionSim(
                              BACK_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                          : new VisionIO() {});
              default -> new Vision(drive, new VisionIO() {}, new VisionIO() {});
            };
      }
    } else vision = null;

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
    configureOperatorButtonBindings();

    // Register the auto commands
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
          Commands.run(
              () ->
                  DriveCommands.joystickDrive(
                      drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()),
              drive));

      // Switch to X pattern when X button is pressed
      driver.stopWithX().onTrue(Commands.runOnce(drive::stopWithX, drive));

      // align to the climb target
      driver.rightAlign().whileTrue(DriveCommands.alignToClimbCommand(drive));
      driver.leftAlign().whileTrue(DriveCommands.alignToClimbCommand(drive));

      driver
          .slowMode()
          .whileTrue(
              Commands.run(
                  () ->
                      DriveCommands.joystickDrive(
                          drive,
                          () -> driver.getYAxis().getAsDouble() / 3.0,
                          () -> driver.getXAxis().getAsDouble() / 3.0,
                          () -> driver.getRotAxis().getAsDouble() / 3.0),
                  drive));

      // // Reset gyro to 0° when B button is pressed
      driver
          .resetOdometry()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.resetOdometry(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
    }
  }

  private void configureOperatorButtonBindings() {
    if (turret == null) {
      return;
    }

    operator
        .turretZero()
        .onTrue(Commands.runOnce(turret::zeroPosition, turret).ignoringDisable(true));

    operator
        .turretPreset()
        .onTrue(TurretCommands.turretToAngle(turret, () -> TurretConstants.PRESET_ANGLE_RAD));

    operator
        .turretManualLeft()
        .whileTrue(TurretCommands.turretOpenLoop(turret, () -> -TurretConstants.MANUAL_PERCENT));

    operator
        .turretManualRight()
        .whileTrue(TurretCommands.turretOpenLoop(turret, () -> TurretConstants.MANUAL_PERCENT));

    if (drive != null && vision != null) {
      operator
          .turretAutoAim()
          .whileTrue(
              TurretCommands.autoAimToTarget(
                  turret, drive::getPose, vision::getBestTargetTranslation));
    }
  }

  /**
   * Use this to pass the autonomwous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous, or null if the auto chooser is not initialized.
   */
  public Command getAutonomousCommand() {
    if (!AUTONOMOUS_ENABLED) return null;
    Command selected = autoChooser.get();
    return selected == autoIdleCommand ? null : selected;
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
      Logger.recordOutput(
          "FieldSimulation/TurretPose", new Pose2d(turretTranslation, turretRotation));
      Logger.recordOutput(
          "FieldSimulation/TurretTarget",
          new Pose2d(TurretConstants.getSimTarget(), new Rotation2d()));
    }
  }
}
