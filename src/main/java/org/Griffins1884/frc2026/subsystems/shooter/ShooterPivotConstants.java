package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.GlobalConstants;

public final class ShooterPivotConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.SPARK_FLEX; // TODO

  public static final int MOTOR_ID = 61; // TODO: set shooter pivot CAN ID
  public static final boolean INVERTED = true;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;

  public static final double FORWARD_LIMIT = 1.0; // TODO: set limits
  public static final double REVERSE_LIMIT = 0.0; // TODO: set limits
  public static final double POSITION_COEFFICIENT = 1.0; // TODO: set gear ratio conversion

  public static final GlobalConstants.Gains GAINS = new GlobalConstants.Gains(4.5, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = 0.0; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = false;
  public static final double SOFT_LIMIT_MIN = REVERSE_LIMIT;
  public static final double SOFT_LIMIT_MAX = FORWARD_LIMIT;
  public static final double MAX_VOLTAGE = 12.0;

  public static final double IDLE_ANGLE_RAD = 0.0; // TODO: set idle angle
  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_START_ANGLE_RAD = 0.0;

  private ShooterPivotConstants() {}
}
