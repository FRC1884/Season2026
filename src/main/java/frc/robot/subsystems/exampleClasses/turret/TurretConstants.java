package frc.robot.subsystems.exampleClasses.turret;

import edu.wpi.first.math.util.Units;
import frc.robot.GlobalConstants;

public final class TurretConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.SPARK_MAX; // TODO
  public static final int MOTOR_ID = 0; // TODO: set turret motor CAN ID
  public static final boolean INVERTED = false; // TODO: set inversion
  public static final int CURRENT_LIMIT_AMPS = 40; // TODO: set current limit
  public static final boolean BRAKE_MODE = true;

  public static final int CANCODER_ID = -1; // TODO: set to enable absolute encoder
  public static final boolean CANCODER_INVERTED = false; // TODO: set CANCoder inversion

  public static final double GEAR_RATIO = 1.0; // TODO: motor rotations per turret rotation
  public static final double ABSOLUTE_ENCODER_GEAR_RATIO =
      1.0; // TODO: absolute encoder rotations per turret rotation
  public static final boolean USE_ABSOLUTE_ENCODER =
      false; // TODO: set true if absolute encoder is available
  public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0.0; // TODO: set absolute offset

  public static final boolean SOFT_LIMITS_ENABLED = true;
  public static final double SOFT_LIMIT_MIN_RAD = Units.degreesToRadians(-170.0); // TODO
  public static final double SOFT_LIMIT_MAX_RAD = Units.degreesToRadians(170.0); // TODO

  public static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.5); // TODO
  public static final double MAX_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(360.0); // TODO
  public static final double MAX_ACCEL_RAD_PER_SEC2 = Units.degreesToRadians(720.0); // TODO

  public static final double MANUAL_PERCENT = 0.2; // TODO: tune manual speed
  public static final double MAX_VOLTAGE = 12.0;

  public static final double kP = 4.0; // TODO: tune
  public static final double kI = 0.0;
  public static final double kD = 0.2; // TODO: tune
  public static final double kS = 0.0; // TODO: tune if using feedforward
  public static final double kV = 0.0; // TODO: tune if using feedforward
  public static final double kA = 0.0; // TODO: tune if using feedforward
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains(kP, kI, kD, kS, kV, kA, 0.0);

  public static final double PRESET_ANGLE_RAD = 0.0; // TODO: set preset angle

  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_MOI = 0.01; // TODO: update if sim is used

  private TurretConstants() {}
}
