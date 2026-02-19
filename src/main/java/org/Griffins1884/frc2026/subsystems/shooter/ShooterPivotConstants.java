package org.Griffins1884.frc2026.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;

public final class ShooterPivotConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.KRAKEN_X40; // TODO

  public static final int[] MOTOR_ID = {7}; // TODO: set shooter pivot CAN ID
  public static final boolean[] INVERTED = {false};
  public static final CANBus CAN_BUS = new CANBus("rio");
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;

  public static final double FORWARD_LIMIT = 1.6;
  public static final double REVERSE_LIMIT = 0.1;
  public static final double POSITION_COEFFICIENT = 1.0; // TODO: set gear ratio conversiond

  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("ShooterPivot/Gains", 1500.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = 0.03; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = true;
  public static final double SOFT_LIMIT_MIN = REVERSE_LIMIT;
  public static final double SOFT_LIMIT_MAX = FORWARD_LIMIT;
  public static final double MAX_VOLTAGE = 12.0;

  public static final double IDLE_ANGLE_RAD = 0.0; // TODO: set idle angle
  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_START_ANGLE_RAD = 0.0;

  public static final double MANUAL_PERCENT = 0.2; // TODO: tune

  private ShooterPivotConstants() {}
}
