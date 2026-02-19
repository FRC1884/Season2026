package org.Griffins1884.frc2026.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;

public final class IntakePivotConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40,
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.KRAKEN_X40; // TODO
  public static final CANBus CAN_BUS = new CANBus("rio"); // TODO: set CAN bus name if needed

  public static final int[] MOTOR_ID = {19, 20}; // TODO: set intake pivot CAN ID
  public static final boolean[] INVERTED = {false, true};
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;

  public static final double FORWARD_LIMIT = 0.0; // TODO: set limits
  public static final double REVERSE_LIMIT = -5.2; // TODO: set limits
  public static final double POSITION_COEFFICIENT = 1.0; // TODO: set gear ratio conversion
  public static final double MOTION_MAGIC_CRUISE_VEL = 1.0; // TODO: tune (position units/sec)
  public static final double MOTION_MAGIC_ACCEL = 2.0; // TODO: tune (position units/sec^2)
  public static final double MOTION_MAGIC_JERK = 0.0; // TODO: tune (position units/sec^3)

  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("IntakePivot/Gains", 500.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = 0.0; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = false;
  public static final double SOFT_LIMIT_MIN = REVERSE_LIMIT;
  public static final double SOFT_LIMIT_MAX = FORWARD_LIMIT;
  public static final double MAX_VOLTAGE = 12.0;

  public static final double IDLE_ANGLE_RAD = 0.0; // TODO: set idle angle
  public static final double PICKUP_RAD = -4.7; // TODO: set idle angle
  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_START_ANGLE_RAD = 0.0;

  private IntakePivotConstants() {}
}
