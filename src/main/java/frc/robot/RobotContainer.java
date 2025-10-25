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
import static frc.robot.Config.Subsystems.*;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.GYRO_TYPE;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.BACK_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.BACK_CAM_ENABLED;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.LEFT_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.LEFT_CAM_ENABLED;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.RIGHT_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.RIGHT_CAM_ENABLED;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config.WebUIConfig;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.auto.AutonomyManager;
import frc.robot.auto.TaskRegistry;
import frc.robot.auto.WebUI;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStates;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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

  // Controller
  private final DriverMap driver = getDriverController();

  private final OperatorMap operator = getOperatorController();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> characterizationChooser;
  private final Command autoIdleCommand;
  private final Command characterizationIdleCommand;

  private final Superstructure superstructure = new Superstructure(null);
  private final Vision vision;
  private final TaskRegistry taskRegistry;
  private final AutonomyManager autonomyManager;
  private final WebUI webUI;

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
                  new ModuleIOSpark(FRONT_LEFT),
                  new ModuleIOSpark(FRONT_RIGHT),
                  new ModuleIOSpark(BACK_LEFT),
                  new ModuleIOSpark(BACK_RIGHT));

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
    }
    if (AUTONOMOUS_ENABLED) {
      if (drive != null) {
        AutoCommands.registerAutoCommands(superstructure, drive);
      }
    }

    if (VISION_ENABLED) {
      vision =
          switch (MODE) {
            case REAL -> new Vision(
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
            case SIM -> new Vision(
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
    } else vision = null;

    TaskRegistry registry = null;
    AutonomyManager manager = null;
    WebUI ui = null;
    if (DRIVETRAIN_ENABLED && drive != null) {
      registry = new TaskRegistry();
      manager = new AutonomyManager(registry, drive, superstructure, 8.0);
      if (WebUIConfig.ENABLED) {
        try {
          ui = new WebUI(registry, WebUIConfig.BIND_ADDRESS, WebUIConfig.PORT);
          ui.start();
          System.out.println(
              "Autonomy WebUI listening on " + WebUIConfig.BIND_ADDRESS + ":" + WebUIConfig.PORT);
        } catch (java.io.IOException e) {
          DriverStation.reportWarning(
              "Failed to start autonomy WebUI on "
                  + WebUIConfig.BIND_ADDRESS
                  + ":"
                  + WebUIConfig.PORT
                  + ": "
                  + e.getMessage(),
              e.getStackTrace());
          ui = null;
        }
      }
    }
    taskRegistry = registry;
    autonomyManager = manager;
    webUI = ui;

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
    }

    superstructure.registerSuperstructureCharacterization(() -> characterizationChooser);

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

      // right align to reef face when right bumper is pressed and the robot is in coral mode
      driver.rightAlign().whileTrue(DriveCommands.rightAlignToReefCommandTeleop(drive));

      // left align to reef face when right numper is pressed and the robot is in coral mode
      driver.leftAlign().whileTrue(DriveCommands.leftAlignToReefCommandTeleop(drive));

      // align to coral station with position customization when right trigger is pressed
      driver
          .coralStation()
          .whileTrue(DriveCommands.alignToNearestCoralStationCommand(drive, driver.getYAxis()));

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

      // Reset gyro to 0° when B button is pressed
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
    operator
        .testing()
        .onTrue(superstructure.setSuperStateCmd(SuperStates.TESTING))
        .onFalse(superstructure.setSuperStateCmd(IDLING));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
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

  public void periodic() {
    if (taskRegistry != null) taskRegistry.pollNetworkControl();
    if (autonomyManager != null) autonomyManager.periodic();
  }

  public TaskRegistry getTaskRegistry() {
    return taskRegistry;
  }

  public AutonomyManager getAutonomyManager() {
    return autonomyManager;
  }

  public WebUI getWebUI() {
    return webUI;
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
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
