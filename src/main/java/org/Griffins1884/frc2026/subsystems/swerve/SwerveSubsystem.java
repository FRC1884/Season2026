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

package org.Griffins1884.frc2026.subsystems.swerve;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.Griffins1884.frc2026.GlobalConstants.RobotMode.SIM;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.PATHPLANNER_CONFIG;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import lombok.Getter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.GlobalConstants.RobotSwerveMotors;
import org.Griffins1884.frc2026.subsystems.vision.Vision;
import org.Griffins1884.frc2026.util.LocalADStarAK;
import org.Griffins1884.frc2026.util.swerve.SwerveSetpoint;
import org.Griffins1884.frc2026.util.swerve.SwerveSetpointGenerator;
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
  private final SwerveMusicPlayer musicPlayer;

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
  @Getter private Rotation2d rawGyroRotation = new Rotation2d();
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
  private String driveSysIdPhase = "IDLE";
  private boolean driveSysIdActive = false;
  private double driveSysIdLastCompleted = Double.NaN;
  private String turnSysIdPhase = "IDLE";
  private boolean turnSysIdActive = false;
  private double turnSysIdLastCompleted = Double.NaN;

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
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS
        && GlobalConstants.MODE != SIM) {
      musicPlayer = new SwerveMusicPlayer(modules, SwerveConstants.SWERVE_MUSIC_FILE);
    } else {
      musicPlayer = null;
    }

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
          // Log per-module telemetry in linear units (meters, m/s).
          for (int i = 0; i < 4; i++) {
            Module module = modules[i];
            log.motor("DriveM" + i)
                .voltage(Volts.of(module.getDriveVoltage()))
                .linearVelocity(MetersPerSecond.of(module.getVelocityMetersPerSec()))
                .linearPosition(Meters.of(module.getPositionMeters()));
          }
        };

    Consumer<SysIdRoutineLog> sysIdLogCallbackTurn =
        (log) -> {
          // Log per-module telemetry in angular units (radians, rad/s).
          for (int i = 0; i < 4; i++) {
            Module module = modules[i];
            log.motor("TurnM" + i)
                .voltage(Volts.of(module.getTurnVoltage()))
                .angularVelocity(RadiansPerSecond.of(module.getTurnVelocityRadPerSec()))
                .angularPosition(Radian.of(module.getTurnPositionRad()));
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
    Logger.recordOutput("Swerve/SysId/DrivePhase", driveSysIdPhase);
    Logger.recordOutput("Swerve/SysId/DriveActive", driveSysIdActive);
    Logger.recordOutput("Swerve/SysId/DriveLastCompleted", driveSysIdLastCompleted);
    Logger.recordOutput("Swerve/SysId/TurnPhase", turnSysIdPhase);
    Logger.recordOutput("Swerve/SysId/TurnActive", turnSysIdActive);
    Logger.recordOutput("Swerve/SysId/TurnLastCompleted", turnSysIdLastCompleted);
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

    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    if (!isFinitePose(estimatedPose)) {
      Rotation2d safeRotation =
          isValidRotation(rawGyroRotation) ? rawGyroRotation : new Rotation2d();
      poseEstimator.resetPosition(safeRotation, getModulePositions(), new Pose2d());
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

  private void setDriveSysIdPhase(String phase, boolean active) {
    driveSysIdPhase = phase;
    driveSysIdActive = active;
    if (!active) {
      driveSysIdLastCompleted = Timer.getFPGATimestamp();
    }
  }

  private void setTurnSysIdPhase(String phase, boolean active) {
    turnSysIdPhase = phase;
    turnSysIdActive = active;
    if (!active) {
      turnSysIdLastCompleted = Timer.getFPGATimestamp();
    }
  }

  public void playSwerveMusic() {
    if (musicPlayer != null) {
      musicPlayer.start();
    }
  }

  public void stopSwerveMusic() {
    if (musicPlayer != null) {
      musicPlayer.stop();
    }
  }

  public void setSwerveMusicVolume(double volume) {
    if (musicPlayer != null) {
      musicPlayer.setVolume(volume);
    }
  }

  public String getDriveSysIdPhase() {
    return driveSysIdPhase;
  }

  public boolean isDriveSysIdActive() {
    return driveSysIdActive;
  }

  public double getDriveSysIdLastCompleted() {
    return driveSysIdLastCompleted;
  }

  public String getTurnSysIdPhase() {
    return turnSysIdPhase;
  }

  public boolean isTurnSysIdActive() {
    return turnSysIdActive;
  }

  public double getTurnSysIdLastCompleted() {
    return turnSysIdLastCompleted;
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
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> runCharacterization(0.0), this)
        .andThen(driveSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> runCharacterization(0.0), this)
        .andThen(driveSysId.dynamic(direction));
  }

  /** Runs the full SysId routine (quasistatic + dynamic, forward + reverse). */
  public Command sysIdRoutine() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Drive Subsystem - Quasistatic (Forward) starting.");
                  setDriveSysIdPhase("QS_FWD", true);
                },
                this),
            sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Drive Subsystem - Quasistatic (Reverse) starting.");
                  setDriveSysIdPhase("QS_REV", true);
                },
                this),
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Drive Subsystem - Dynamic (Forward) starting.");
                  setDriveSysIdPhase("DYN_FWD", true);
                },
                this),
            sysIdDynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Drive Subsystem - Dynamic (Reverse) starting.");
                  setDriveSysIdPhase("DYN_REV", true);
                },
                this),
            sysIdDynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(
                () -> {
                  runCharacterization(0.0);
                  setDriveSysIdPhase("DONE", false);
                },
                this))
        .withName("DriveSysIdRoutine");
  }

  /** Returns a command to run a steer-motor quasistatic test. */
  public Command sysIdTurnQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> runTurnCharacterization(0.0), this)
        .andThen(turnSysId.quasistatic(direction));
  }

  /** Returns a command to run a steer-motor dynamic test. */
  public Command sysIdTurnDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> runTurnCharacterization(0.0), this)
        .andThen(turnSysId.dynamic(direction));
  }

  /** Runs the full SysId routine for the steer motors. */
  public Command sysIdTurnRoutine() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  System.out.println(
                      "[SysId] Turn Subsystem - Quasistatic (Forward) starting.");
                  setTurnSysIdPhase("QS_FWD", true);
                },
                this),
            sysIdTurnQuasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println(
                      "[SysId] Turn Subsystem - Quasistatic (Reverse) starting.");
                  setTurnSysIdPhase("QS_REV", true);
                },
                this),
            sysIdTurnQuasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Turn Subsystem - Dynamic (Forward) starting.");
                  setTurnSysIdPhase("DYN_FWD", true);
                },
                this),
            sysIdTurnDynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            Commands.runOnce(
                () -> {
                  System.out.println("[SysId] Turn Subsystem - Dynamic (Reverse) starting.");
                  setTurnSysIdPhase("DYN_REV", true);
                },
                this),
            sysIdTurnDynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(
                () -> {
                  runTurnCharacterization(0.0);
                  setTurnSysIdPhase("DONE", false);
                },
                this))
        .withName("TurnSysIdRoutine");
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
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

  /**
   * Zeros the gyro and odometry heading to the alliance wall.
   *
   * <p>Red alliance: facing the red wall = 0 degrees. Blue alliance: facing the blue wall = 180
   * degrees (WPI blue field coordinates).
   */
  public void zeroGyroAndOdometryToAllianceWall(Alliance alliance) {
    Rotation2d heading = getAllianceWallFacingRotation(alliance);
    resetOdometry(new Pose2d(getPose().getTranslation(), heading), true);
    Logger.recordOutput("Odometry/AllianceZero/HeadingDeg", heading.getDegrees());
    Logger.recordOutput("Odometry/AllianceZero/Alliance", alliance.name());
  }

  /** Returns the field heading used when the robot is facing its alliance wall. */
  public static Rotation2d getAllianceWallFacingRotation(Alliance alliance) {
    return alliance == Alliance.Blue ? Rotation2d.fromDegrees(180.0) : new Rotation2d();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    resetOdometry(pose, false);
  }

  /**
   * Resets the current odometry pose and optionally aligns the gyro to the provided field heading.
   *
   * @param pose Field-relative pose to reset to.
   * @param resetGyro If true, reset the gyro yaw to pose rotation (field heading).
   */
  public void resetOdometry(Pose2d pose, boolean resetGyro) {
    if (resetGyro) {
      gyroIO.resetYaw(pose.getRotation().getDegrees());
      rawGyroRotation = pose.getRotation();
    }
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (visionRobotPoseMeters == null || visionMeasurementStdDevs == null) {
      return;
    }
    if (!isFinitePose(visionRobotPoseMeters) || !isFiniteMatrix(visionMeasurementStdDevs)) {
      return;
    }

    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    if (!isFinitePose(currentPose)) {
      if (!isValidRotation(rawGyroRotation)) {
        // rawGyroRotation = visionRobotPoseMeters.getRotation();
      }
      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), visionRobotPoseMeters);
      return;
    }

    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  private static boolean isFinitePose(Pose2d pose) {
    if (pose == null) {
      return false;
    }
    if (!isFinite(pose.getX()) || !isFinite(pose.getY())) {
      return false;
    }
    return isValidRotation(pose.getRotation());
  }

  private static boolean isValidRotation(Rotation2d rotation) {
    if (rotation == null) {
      return false;
    }
    double cos = rotation.getCos();
    double sin = rotation.getSin();
    if (!isFinite(cos) || !isFinite(sin)) {
      return false;
    }
    return !(Math.abs(cos) < 1e-9 && Math.abs(sin) < 1e-9);
  }

  private static boolean isFiniteMatrix(Matrix<N3, N1> matrix) {
    if (matrix == null) {
      return false;
    }
    for (int row = 0; row < 3; row++) {
      if (!isFinite(matrix.get(row, 0))) {
        return false;
      }
    }
    return true;
  }

  private static boolean isFinite(double value) {
    return Double.isFinite(value);
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
