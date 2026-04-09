package org.Griffins1884.frc2026.subsystems.swerve;

import static java.lang.Math.PI;
import static org.Griffins1884.frc2026.GlobalConstants.ROBOT;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.Griffins1884.frc2026.GlobalConstants.Gains;
import org.Griffins1884.frc2026.GlobalConstants.RobotType;
import org.Griffins1884.frc2026.util.swerve.ModuleLimits;

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
  public static final double TRACK_WIDTH =
      switch (ROBOT) {
        case ECLAIR -> Units.inchesToMeters(13.0);
        case COMPBOT, DBOT, SIMBOT -> Units.inchesToMeters(27.5);
      };

  /** Meters */
  public static final double WHEEL_BASE =
      switch (ROBOT) {
        case ECLAIR -> Units.inchesToMeters(42.0);
        case COMPBOT, DBOT, SIMBOT -> Units.inchesToMeters(27.5);
      };

  /** Meters */
  public static final double BUMPER_LENGTH =
      switch (ROBOT) {
        case ECLAIR -> Units.inchesToMeters(42.0);
        case COMPBOT, DBOT, SIMBOT -> Units.inchesToMeters(34.0);
      };

  /** Meters */
  public static final double BUMPER_WIDTH = Units.inchesToMeters(34.0);

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

  // TODO: Replace all ECLAIR CAN IDs with the real hardware assignments.
  private static final int ECLAIR_PIGEON_ID = -1;
  private static final int ECLAIR_FRD_ID = -1;
  private static final int ECLAIR_FRR_ID = -1;
  private static final int ECLAIR_FRR_CANCODER_ID = -1;
  private static final int ECLAIR_FLD_ID = -1;
  private static final int ECLAIR_FLR_ID = -1;
  private static final int ECLAIR_FLR_CANCODER_ID = -1;
  private static final int ECLAIR_BRD_ID = -1;
  private static final int ECLAIR_BRR_ID = -1;
  private static final int ECLAIR_BRR_CANCODER_ID = -1;
  private static final int ECLAIR_BLD_ID = -1;
  private static final int ECLAIR_BLR_ID = -1;
  private static final int ECLAIR_BLR_CANCODER_ID = -1;

  static final int PIGEON_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 60;
        case ECLAIR -> ECLAIR_PIGEON_ID;
      };
  ;
  private static final int FRD_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 13;
        case ECLAIR -> ECLAIR_FRD_ID;
      };
  private static final int FRR_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 12;
        case ECLAIR -> ECLAIR_FRR_ID;
      };
  private static final int FRR_CANCODER_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 4;
        case ECLAIR -> ECLAIR_FRR_CANCODER_ID;
      };
  private static final int FLD_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 11;
        case ECLAIR -> ECLAIR_FLD_ID;
      };
  private static final int FLR_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 10;
        case ECLAIR -> ECLAIR_FLR_ID;
      };
  private static final int FLR_CANCODER_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 3;
        case ECLAIR -> ECLAIR_FLR_CANCODER_ID;
      };
  private static final int BRD_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 14;
        case ECLAIR -> ECLAIR_BRD_ID;
      };
  private static final int BRR_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 15;
        case ECLAIR -> ECLAIR_BRR_ID;
      };
  private static final int BRR_CANCODER_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 5;
        case ECLAIR -> ECLAIR_BRR_CANCODER_ID;
      };
  private static final int BLD_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 16;
        case ECLAIR -> ECLAIR_BLD_ID;
      };
  private static final int BLR_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 17;
        case ECLAIR -> ECLAIR_BLR_ID;
      };
  private static final int BLR_CANCODER_ID =
      switch (ROBOT) {
        case COMPBOT, DBOT, SIMBOT -> 6;
        case ECLAIR -> ECLAIR_BLR_CANCODER_ID;
      };

  // Zeroed rotation values for each module, see setup instructions
  private static final Rotation2d COMPBOT_FLR_ZERO =
      Rotation2d.fromRadians(-0.482666015625 * (2 * PI));
  private static final Rotation2d COMPBOT_FRR_ZERO =
      Rotation2d.fromRadians(-0.111328125 * (2 * PI));
  private static final Rotation2d COMPBOT_BLR_ZERO = Rotation2d.fromRadians(0.00390625 * (2 * PI));
  private static final Rotation2d COMPBOT_BRR_ZERO =
      Rotation2d.fromRadians(-0.290283203125 * (2 * PI));

  // TODO: Measure the absolute encoder zero offsets for ECLAIR.
  private static final Rotation2d ECLAIR_FLR_ZERO = Rotation2d.fromRadians(0.0);
  private static final Rotation2d ECLAIR_FRR_ZERO = Rotation2d.fromRadians(0.0);
  private static final Rotation2d ECLAIR_BLR_ZERO = Rotation2d.fromRadians(0.0);
  private static final Rotation2d ECLAIR_BRR_ZERO = Rotation2d.fromRadians(0.0);

  private static final Rotation2d FLR_ZERO =
      switch (ROBOT) {
        case ECLAIR -> ECLAIR_FLR_ZERO;
        case COMPBOT, DBOT, SIMBOT -> COMPBOT_FLR_ZERO;
      };
  private static final Rotation2d FRR_ZERO =
      switch (ROBOT) {
        case ECLAIR -> ECLAIR_FRR_ZERO;
        case COMPBOT, DBOT, SIMBOT -> COMPBOT_FRR_ZERO;
      };
  private static final Rotation2d BLR_ZERO =
      switch (ROBOT) {
        case ECLAIR -> ECLAIR_BLR_ZERO;
        case COMPBOT, DBOT, SIMBOT -> COMPBOT_BLR_ZERO;
      };
  private static final Rotation2d BRR_ZERO =
      switch (ROBOT) {
        case ECLAIR -> ECLAIR_BRR_ZERO;
        case COMPBOT, DBOT, SIMBOT -> COMPBOT_BRR_ZERO;
      };

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
  public static final double NOMINAL_WHEEL_RADIUS = Units.inchesToMeters(2);

  /** Wheel rotations induced per full steering rotation at the motor sensor. */
  public static final double KRAKEN_STEER_DRIVE_COUPLING_RATIO = 4.5;

  /** Meters per second */
  public static final double MAX_LINEAR_SPEED =
      (ROBOT == RobotType.COMPBOT || ROBOT == RobotType.ECLAIR) ? 5.4804 : 5.3;

  /** Radians per second */
  public static final double MAX_ANGULAR_SPEED = (0.5 * MAX_LINEAR_SPEED) / DRIVE_BASE_RADIUS;

  /** Meters per second squared */
  public static final double MAX_LINEAR_ACCELERATION = 12.0;

  /** Radians per second */
  public static final double MAX_STEERING_VELOCITY = Units.degreesToRadians(1080.0);

  public static final double WHEEL_FRICTION_COEFF = 1.25;

  /** Kilograms per square meter */
  public static final double ROBOT_INERTIA = 6.883;

  /** Kilograms */
  public static final double ROBOT_MASS = Units.lbsToKilograms(135.0);

  // Drive motor configuration
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60(1);

  public static final double DRIVE_GEAR_RATIO = 5.08; // Spark Max
  public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.03; // MK5n drive ratio (per team)

  static final boolean DRIVE_INVERTED = true;

  /** Amps */
  static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

  static final int KRAKEN_DRIVE_CURRENT_LIMIT = 40;

  /** Amps */
  static final double DRIVE_MOTOR_MAX_TORQUE = DRIVE_GEARBOX.getTorque(DRIVE_MOTOR_CURRENT_LIMIT);

  // Drive motor PID configuration
  static final Gains DRIVE_MOTOR_GAINS =
      switch (ROBOT) {
        case COMPBOT, DBOT, ECLAIR ->
            new Gains("Swerve/DriveMotor/Compbot", 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
        case SIMBOT -> new Gains("Swerve/DriveMotor/Simbot", 0.05, 0.0, 0.0, 0.0, 0.0789, 0.0, 0.0);
      };
  // Torque-current gains for Kraken FOC (amps-based, per-radian units)
  static final Gains KRAKEN_DRIVE_TORQUE_GAINS =
      switch (ROBOT) {
        case COMPBOT, DBOT, ECLAIR ->
            new Gains("Swerve/KrakenDriveTorque/Compbot", 45.0, 0.0, 0.0, 5.0, 0.4, 0.0, 0.0);
        case SIMBOT ->
            new Gains("Swerve/KrakenDriveTorque/Simbot", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  // Drive encoder configuration
  /** Wheel radians */
  static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_GEAR_RATIO;

  /** Wheel radians per second */
  static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;

  // Rotator motor configuration
  public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60(1);

  public static final double ROTATOR_GEAR_RATIO = 9424.0 / 203.0;
  public static final double KRAKEN_ROTATOR_GEAR_RATIO = 287.0 / 11.0;

  /** Amps */
  static final int ROTATOR_MOTOR_CURRENT_LIMIT_AMPS = 20;

  static final int KRAKEN_ROTATOR_CURRENT_LIMIT_AMPS = 20;

  static final boolean ROTATOR_INVERTED = false;

  // Rotator PID configuration
  static final Gains ROTATOR_GAINS =
      switch (ROBOT) {
        case COMPBOT, DBOT, ECLAIR -> new Gains("Swerve/Rotator/Compbot", 2.0, 0.0, 0.0);
        case SIMBOT -> new Gains("Swerve/Rotator/Simbot", 12.0, 0.0, 0.2);
      };
  // Torque-current gains for Kraken turn control (amps-based, per-radian units)
  static final Gains KRAKEN_TURN_TORQUE_GAINS =
      switch (ROBOT) {
        case COMPBOT, DBOT, ECLAIR ->
            new Gains("Swerve/KrakenTurnTorque/Compbot", 8000.0, 0.0, 50.0);
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
  public static final ClosedLoopGains TRANSLATION_CONSTANTS = new ClosedLoopGains(10, 0, 0);
  public static final ClosedLoopGains ROTATION_CONSTANTS = new ClosedLoopGains(100, 0, 0);

  // Mechanical Advantage-style module limits (used for FULLKRACKENS)
  public static final ModuleLimits KRAKEN_MODULE_LIMITS_FREE =
      new ModuleLimits(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_STEERING_VELOCITY);

  public static double getWheelRadiusMeters() {
    return SwerveCalibration.getWheelRadiusMeters(NOMINAL_WHEEL_RADIUS);
  }

  public static double getCouplingWheelRadiansPerSteerRadian() {
    return KRAKEN_STEER_DRIVE_COUPLING_RATIO / KRAKEN_DRIVE_GEAR_RATIO;
  }

  public record ClosedLoopGains(double kP, double kI, double kD) {}
}
