package org.Griffins1884.frc2026.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.PI;
import static org.Griffins1884.frc2026.GlobalConstants.ROBOT;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.GlobalConstants.Gains;
import org.Griffins1884.frc2026.GlobalConstants.RobotSwerveMotors;
import org.Griffins1884.frc2026.GlobalConstants.RobotType;
import org.Griffins1884.frc2026.util.swerve.ModuleLimits;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class SwerveConstants {
  // Gyro
  public static enum GyroType {
    PIGEON,
    NAVX,
    ADIS,
  }

  public static final GyroType GYRO_TYPE = GyroType.PIGEON;

  // Swerve music (Kraken-only).
  // CTRE Orchestra expects a .chrp file (place it under src/main/deploy/music).
  public static final String SWERVE_MUSIC_FILE = "music/swerve.chrp";

  /** Meters */
  public static final double TRACK_WIDTH = Units.inchesToMeters(27.5);

  /** Meters */
  public static final double WHEEL_BASE = Units.inchesToMeters(27.5);

  /** Meters */
  public static final double BUMPER_LENGTH = Units.inchesToMeters(34);

  /** Meters */
  public static final double BUMPER_WIDTH = Units.inchesToMeters(34);

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)
      };

  /** Meters */
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

  /**
   * Represents each module's constants on a NEO-based swerve.
   *
   * @param name of the module, for logging purposes
   * @param driveID
   * @param rotatorID
   * @param zeroRotation in radians
   */
  public record ModuleConstants(
      String name,
      int driveID,
      int rotatorID,
      int cancoderID,
      Rotation2d zeroRotation,
      boolean turnInverted,
      boolean encoderInverted) {}

  static final int PIGEON_ID =
      switch (ROBOT) {
        case DEVBOT -> 60;
        case COMPBOT, SIMBOT -> 60;
        case CRESCENDO -> 40;
      };
  ;
  private static final int FRD_ID =
      switch (ROBOT) {
        case DEVBOT -> 14;
        case COMPBOT, SIMBOT -> 11;
        case CRESCENDO -> 43;
      };
  private static final int FRR_ID =
      switch (ROBOT) {
        case DEVBOT -> 15;
        case COMPBOT, SIMBOT -> 21;
        case CRESCENDO -> 5;
      };
  private static final int FRR_CANCODER_ID =
      switch (ROBOT) {
        case DEVBOT -> 5;
        case COMPBOT, SIMBOT -> -1;
        case CRESCENDO -> -1;
      };
  private static final int FLD_ID =
      switch (ROBOT) {
        case DEVBOT -> 13;
        case COMPBOT, SIMBOT -> 12;
        case CRESCENDO -> 41;
      };
  private static final int FLR_ID =
      switch (ROBOT) {
        case DEVBOT -> 12;
        case COMPBOT, SIMBOT -> 22;
        case CRESCENDO -> 1;
      };
  private static final int FLR_CANCODER_ID =
      switch (ROBOT) {
        case DEVBOT -> 4;
        case COMPBOT, SIMBOT -> -1;
        case CRESCENDO -> -1;
      };
  private static final int BRD_ID =
      switch (ROBOT) {
        case DEVBOT -> 16;
        case COMPBOT, SIMBOT -> 14;
        case CRESCENDO -> 40;
      };
  private static final int BRR_ID =
      switch (ROBOT) {
        case DEVBOT -> 17;
        case COMPBOT, SIMBOT -> 24;
        case CRESCENDO -> 7;
      };
  private static final int BRR_CANCODER_ID =
      switch (ROBOT) {
        case DEVBOT -> 6;
        case COMPBOT, SIMBOT -> -1;
        case CRESCENDO -> -1;
      };
  private static final int BLD_ID =
      switch (ROBOT) {
        case DEVBOT -> 11;
        case COMPBOT, SIMBOT -> 13;
        case CRESCENDO -> 31;
      };
  private static final int BLR_ID =
      switch (ROBOT) {
        case DEVBOT -> 10;
        case COMPBOT, SIMBOT -> 23;
        case CRESCENDO -> 3;
      };
  private static final int BLR_CANCODER_ID =
      switch (ROBOT) {
        case DEVBOT -> 3;
        case COMPBOT, SIMBOT -> -1;
        case CRESCENDO -> -1;
      };

  // Zeroed rotation values for each module, see setup instructions
  private static final Rotation2d FLR_ZERO =
      (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS)
          ? Rotation2d.fromRadians(0.075927734375 * (2 * PI))
          : Rotation2d.fromRadians(-PI / 2);
  private static final Rotation2d FRR_ZERO =
      (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS)
          ? Rotation2d.fromRadians(-0.204345703125 * (2 * PI))
          : Rotation2d.fromRadians(0);
  private static final Rotation2d BLR_ZERO =
      (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS)
          ? Rotation2d.fromRadians(0.253173828125 * (2 * PI))
          : Rotation2d.fromRadians(PI);
  private static final Rotation2d BRR_ZERO =
      (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS)
          ? Rotation2d.fromRadians(0.432373046875 * (2 * PI))
          : Rotation2d.fromRadians(PI / 2);

  // Inverted encoders or turn motors
  private static final boolean FLR_INVERTED = false;
  private static final boolean FLR_ENCODER_INVERTED = false;
  private static final boolean FRR_INVERTED = false;
  private static final boolean FRR_ENCODER_INVERTED = false;
  private static final boolean BLR_INVERTED = false;
  private static final boolean BLR_ENCODER_INVERTED = false;
  private static final boolean BRR_INVERTED = false;
  private static final boolean BRR_ENCODER_INVERTED = false;

  // Constants for each module. Add the CANCoder id between the rotator id and offset params
  public static final ModuleConstants FRONT_LEFT =
      new ModuleConstants(
          "Front Left",
          FLD_ID,
          FLR_ID,
          FLR_CANCODER_ID,
          FLR_ZERO,
          FLR_INVERTED,
          FLR_ENCODER_INVERTED);
  public static final ModuleConstants FRONT_RIGHT =
      new ModuleConstants(
          "Front Right",
          FRD_ID,
          FRR_ID,
          FRR_CANCODER_ID,
          FRR_ZERO,
          FRR_INVERTED,
          FRR_ENCODER_INVERTED);
  public static final ModuleConstants BACK_LEFT =
      new ModuleConstants(
          "Back Left",
          BLD_ID,
          BLR_ID,
          BLR_CANCODER_ID,
          BLR_ZERO,
          BLR_INVERTED,
          BLR_ENCODER_INVERTED);
  public static final ModuleConstants BACK_RIGHT =
      new ModuleConstants(
          "Back Right",
          BRD_ID,
          BRR_ID,
          BRR_CANCODER_ID,
          BRR_ZERO,
          BRR_INVERTED,
          BRR_ENCODER_INVERTED);

  /** Meters */
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

  /** Meters per second */
  public static final double MAX_LINEAR_SPEED = 5.4804;

  /** Radians per second */
  public static final double MAX_ANGULAR_SPEED = (0.5 * MAX_LINEAR_SPEED) / DRIVE_BASE_RADIUS;

  /** Meters per second squared */
  public static final double MAX_LINEAR_ACCELERATION = 22.0;

  /** Radians per second */
  public static final double MAX_STEERING_VELOCITY = Units.degreesToRadians(1080.0);

  public static final double WHEEL_FRICTION_COEFF = COTS.WHEELS.SLS_PRINTED_WHEELS.cof;
  private static final double MAPLE_SIM_WHEEL_FRICTION_COEFF = Math.min(WHEEL_FRICTION_COEFF, 1.5);

  /** Kilograms per square meter */
  public static final double ROBOT_INERTIA = 6.883;

  /** Kilograms */
  public static final double ROBOT_MASS = 45;

  // Drive motor configuration
  public static final DCMotor DRIVE_GEARBOX =
      (GlobalConstants.robotSwerveMotors == GlobalConstants.RobotSwerveMotors.FULLSPARK)
          ? DCMotor.getNeoVortex(1)
          : DCMotor.getKrakenX60(1);

  public static final double DRIVE_GEAR_RATIO = 5.08; // Spark Max
  public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.03; // MK5n drive ratio (per team)

  static final boolean DRIVE_INVERTED = ROBOT == RobotType.DEVBOT;

  /** Amps */
  static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

  static final int KRAKEN_DRIVE_CURRENT_LIMIT = 40;

  /** Amps */
  static final double DRIVE_MOTOR_MAX_TORQUE = DRIVE_GEARBOX.getTorque(DRIVE_MOTOR_CURRENT_LIMIT);

  // Drive motor PID configuration
  static final Gains DRIVE_MOTOR_GAINS =
      switch (ROBOT) {
        case COMPBOT -> new Gains("Swerve/DriveMotor/Compbot", 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
        case SIMBOT -> new Gains("Swerve/DriveMotor/Simbot", 0.05, 0.0, 0.0, 0.0, 0.0789, 0.0, 0.0);
        case DEVBOT ->
            new Gains(
                "Swerve/DriveMotor/Devbot", 1.0629, 0.0, 0.0, 0.23397, 0.72165, 0.039375, 0.0);
        case CRESCENDO ->
            new Gains(
                "Swerve/DriveMotor/Crescendo",
                0.0023439,
                0.0,
                0.0,
                0.10953,
                0.54241,
                0.084571,
                0.0);
      };
  // Torque-current gains for Kraken FOC (amps-based, per-radian units)
  static final Gains KRAKEN_DRIVE_TORQUE_GAINS =
      switch (ROBOT) {
        case COMPBOT ->
            new Gains("Swerve/KrakenDriveTorque/Compbot", 45.0, 0.0, 0.0, 5.0, 0.4, 0.0, 0.0);
        case DEVBOT ->
            new Gains("Swerve/KrakenDriveTorque/Devbot", 45.0, 0.0, 0.0, 5.0, 0.4, 0.0, 0.0);
        case CRESCENDO ->
            new Gains("Swerve/KrakenDriveTorque/Crescendo", 45.0, 0.0, 0.0, 5.0, 0.4, 0.0, 0.0);
        case SIMBOT ->
            new Gains("Swerve/KrakenDriveTorque/Simbot", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  // Drive encoder configuration
  /** Wheel radians */
  static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_GEAR_RATIO;

  /** Wheel radians per second */
  static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;

  // Rotator motor configuration
  public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);

  public static final double ROTATOR_GEAR_RATIO = 9424.0 / 203.0;
  public static final double KRAKEN_ROTATOR_GEAR_RATIO = 287.0 / 11.0;

  /** Amps */
  static final int ROTATOR_MOTOR_CURRENT_LIMIT_AMPS = 20;

  static final int KRAKEN_ROTATOR_CURRENT_LIMIT_AMPS = 20;

  static final boolean ROTATOR_INVERTED = false;

  // Rotator PID configuration
  static final Gains ROTATOR_GAINS =
      switch (ROBOT) {
        case DEVBOT -> new Gains("Swerve/Rotator/Devbot", 2.0, 0.0, 0.0);
        case COMPBOT -> new Gains("Swerve/Rotator/Compbot", 2.0, 0.0, 0.0);
        case SIMBOT -> new Gains("Swerve/Rotator/Simbot", 8.0, 0.0, 0.0);
        case CRESCENDO -> new Gains("Swerve/Rotator/Crescendo", 2.0, 0.0, 0.0);
      };
  // Torque-current gains for Kraken turn control (amps-based, per-radian units)
  static final Gains KRAKEN_TURN_TORQUE_GAINS =
      switch (ROBOT) {
        case DEVBOT -> new Gains("Swerve/KrakenTurnTorque/Devbot", 8000.0, 0.0, 50.0);
        case COMPBOT -> new Gains("Swerve/KrakenTurnTorque/Compbot", 8000.0, 0.0, 50.0);
        case CRESCENDO -> new Gains("Swerve/KrakenTurnTorque/Crescendo", 8000.0, 0.0, 50.0);
        case SIMBOT -> new Gains("Swerve/KrakenTurnTorque/Simbot", 0.0, 0.0, 0.0);
      };

  /** CanBus */
  static final boolean canivore = true;

  static final CANBus canBus = new CANBus("DriveTrain");

  /** Radians */
  static final double ROTATOR_PID_MIN_INPUT = 0;

  /** Radians */
  static final double ROTATOR_PID_MAX_INPUT = 2 * Math.PI;

  /** Degrees */
  static final double TURN_DEADBAND_DEGREES = 0.3;

  // Rotator encoder configuration
  static final boolean ROTATOR_ENCODER_INVERTED = true;

  /** Radians */
  static final double ROTATOR_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians

  /** Radians per second */
  static final double ROTATOR_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Drivetrain PID
  public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(10, 0, 0);
  public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(100, 0, 0);

  // Mechanical Advantage-style module limits (used for FULLKRACKENS)
  public static final ModuleLimits KRAKEN_MODULE_LIMITS_FREE =
      new ModuleLimits(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_STEERING_VELOCITY);

  // PathPlanner configuration
  public static final RobotConfig PATHPLANNER_CONFIG =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_INERTIA,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_LINEAR_SPEED,
              WHEEL_FRICTION_COEFF,
              DRIVE_GEARBOX.withReduction(DRIVE_GEAR_RATIO),
              DRIVE_MOTOR_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      new DriveTrainSimulationConfig(
          Kilograms.of(ROBOT_MASS),
          Meters.of(BUMPER_LENGTH),
          Meters.of(BUMPER_WIDTH),
          Meters.of(WHEEL_BASE),
          Meters.of(TRACK_WIDTH),
          switch (GYRO_TYPE) {
            case PIGEON -> COTS.ofPigeon2();
            case NAVX -> COTS.ofNav2X();
            case ADIS -> COTS.ofGenericGyro();
          },
          () ->
              COTS.ofMAXSwerve(
                      DRIVE_GEARBOX, // Drive motor is a Neo Vortex
                      TURN_GEARBOX, // Steer motor is a Neo 550
                      MAPLE_SIM_WHEEL_FRICTION_COEFF, // Clamp for sim stability
                      2) // Medium Gear ratio
                  .get());
}
