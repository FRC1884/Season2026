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

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.RobotMode.SIM;
import static frc.robot.subsystems.swerve.SwerveConstants.PATHPLANNER_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotSwerveMotors;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase implements Vision.VisionConsumer {
  private static final double DRIVE_SYS_ID_MAX_VOLTAGE = 40.0;
  private static final double TURN_SYS_ID_MAX_VOLTAGE = 12.0;
  private static final double SYS_ID_IDLE_WAIT_SECONDS = 0.5;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine driveSysId;
  private final SysIdRoutine turnSysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private final SwerveSetpointGenerator krakenSetpointGenerator =
      new SwerveSetpointGenerator(kinematics, SwerveConstants.MODULE_TRANSLATIONS);
  private SwerveSetpoint krakenCurrentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private boolean krakenVelocityMode = false;

  public SwerveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS)
      PhoenixOdometryThread.getInstance().start();
    else SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PATHPLANNER_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    Consumer<SysIdRoutineLog> sysIdLogCallbackDrive =
        (log) -> {
          // Log per-module telemetry
          for (int i = 0; i < 4; i++) {
            Module module = modules[i];
            log.motor("DriveM" + i)
                .voltage(Volts.of(module.getVoltage()))
                .angularVelocity(RadiansPerSecond.of(module.getFFCharacterizationVelocity()))
                .angularPosition(Radian.of(module.getWheelRadiusCharacterizationPosition()));
          }
        };

    Consumer<SysIdRoutineLog> sysIdLogCallbackTurn =
        (log) -> {
          // Log per-module telemetry
          for (int i = 0; i < 4; i++) {
            Module module = modules[i];
            log.motor("TurnM" + i)
                .voltage(Volts.of(module.getVoltage()))
                .angularVelocity(RadiansPerSecond.of(module.getFFCharacterizationVelocity()))
                .angularPosition(Radian.of(module.getWheelRadiusCharacterizationPosition()));
          }
        };

    // Configure drive SysId
    driveSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(2.5),
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDriveSysIdVoltage(voltage.in(Volts)), sysIdLogCallbackDrive, this));

    // Configure turn SysId
    turnSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(2.5),
                (state) -> Logger.recordOutput("Drive/TurnSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runTurnSysIdVoltage(voltage.in(Volts)), sysIdLogCallbackTurn, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    int gyroSampleCount = gyroInputs.odometryYawPositions.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation =
            i < gyroSampleCount ? gyroInputs.odometryYawPositions[i] : rawGyroRotation;
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS
        && !krakenVelocityMode) {
      krakenCurrentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && GlobalConstants.MODE != SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      krakenVelocityMode = true;
      ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      SwerveModuleState[] setpointStatesUnoptimized =
          kinematics.toSwerveModuleStates(discreteSpeeds);
      krakenCurrentSetpoint =
          krakenSetpointGenerator.generateSetpoint(
              SwerveConstants.KRAKEN_MODULE_LIMITS_FREE,
              krakenCurrentSetpoint,
              discreteSpeeds,
              0.02);
      SwerveModuleState[] setpointStates = krakenCurrentSetpoint.moduleStates();

      Logger.recordOutput("SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", krakenCurrentSetpoint.chassisSpeeds());

      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }

      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
      return;
    }

    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_LINEAR_SPEED);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      krakenVelocityMode = false;
    }
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Runs the turn motors open-loop for SysId and tuning. */
  public void runTurnCharacterization(double output) {
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      krakenVelocityMode = false;
    }
    for (int i = 0; i < 4; i++) {
      modules[i].runTurnCharacterization(output);
    }
  }

  private void runDriveSysIdVoltage(double voltage) {
    runCharacterization(
        MathUtil.clamp(voltage, -DRIVE_SYS_ID_MAX_VOLTAGE, DRIVE_SYS_ID_MAX_VOLTAGE));
  }

  private void runTurnSysIdVoltage(double voltage) {
    runTurnCharacterization(
        MathUtil.clamp(voltage, -TURN_SYS_ID_MAX_VOLTAGE, TURN_SYS_ID_MAX_VOLTAGE));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = SwerveConstants.MODULE_TRANSLATIONS[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      // Bypass setpoint generator to lock wheels in an X.
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
      for (int i = 0; i < 4; i++) {
        states[i].optimize(modules[i].getAngle());
        modules[i].runSetpoint(states[i]);
      }
    } else {
      stop();
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(driveSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(driveSysId.dynamic(direction));
  }

  /** Runs the full SysId routine (quasistatic + dynamic, forward + reverse). */
  public Command sysIdRoutine() {
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    System.out.println("[SysId] Drive Subsystem - Quasistatic (Forward) starting."),
                this),
            sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () ->
                    System.out.println("[SysId] Drive Subsystem - Quasistatic (Reverse) starting."),
                this),
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> System.out.println("[SysId] Drive Subsystem - Dynamic (Forward) starting."),
                this),
            sysIdDynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> System.out.println("[SysId] Drive Subsystem - Dynamic (Reverse) starting."),
                this),
            sysIdDynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> runCharacterization(0.0), this))
        .withName("DriveSysIdRoutine");
  }

  /** Returns a command to run a steer-motor quasistatic test. */
  public Command sysIdTurnQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(turnSysId.quasistatic(direction));
  }

  /** Returns a command to run a steer-motor dynamic test. */
  public Command sysIdTurnDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(turnSysId.dynamic(direction));
  }

  /** Runs the full SysId routine for the steer motors. */
  public Command sysIdTurnRoutine() {
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    System.out.println("[SysId] Turn Subsystem - Quasistatic (Forward) starting."),
                this),
            sysIdTurnQuasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () ->
                    System.out.println("[SysId] Turn Subsystem - Quasistatic (Reverse) starting."),
                this),
            sysIdTurnQuasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> System.out.println("[SysId] Turn Subsystem - Dynamic (Forward) starting."),
                this),
            sysIdTurnDynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> System.out.println("[SysId] Turn Subsystem - Dynamic (Reverse) starting."),
                this),
            sysIdTurnDynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> runTurnCharacterization(0.0), this))
        .withName("TurnSysIdRoutine");
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Distance/Robot distance to face")
  public double targetFaceDistance() {
    Supplier<Pose2d> target = DriveCommands.findClosestReefFace(this);
    return target.get().minus(getPose()).getX();
  }

  @AutoLogOutput(key = "Distance/Reef Target")
  public Pose2d targetReefFace() {
    return DriveCommands.findClosestReefFace(this).get();
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the current robot-relative chassis speeds. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getChassisSpeeds();
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current yaw rate in degrees per second from the gyro. */
  public double getYawRateDegreesPerSec() {
    return Math.toDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return SwerveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return SwerveConstants.MAX_ANGULAR_SPEED;
  }
}
