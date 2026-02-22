package org.Griffins1884.frc2026.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

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

  public static final double FORWARD_LIMIT = 5.0; // TODO: set limits
  public static final double REVERSE_LIMIT = -12.0; // TODO: set limits
  public static final double POSITION_COEFFICIENT = 1.0; // TODO: set gear ratio conversion
  public static final LoggedTunableNumber MOTION_MAGIC_CRUISE_VEL =
      new LoggedTunableNumber("IntakePivot/MotionMagic/CruiseVel", 10.0);
  public static final LoggedTunableNumber MOTION_MAGIC_ACCEL =
      new LoggedTunableNumber("IntakePivot/MotionMagic/Accel", 10.0);
  public static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber("IntakePivot/MotionMagic/Jerk", 10.0);

  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("IntakePivot/Gains", 450.0, 55.0, 60.0, 0.0, 0., 0.0, 3);
  public static final double POSITION_TOLERANCE = 0.0; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = true;
  public static final double SOFT_LIMIT_MIN = REVERSE_LIMIT;
  public static final double SOFT_LIMIT_MAX = FORWARD_LIMIT;
  public static final double MAX_VOLTAGE = 12.0;

  public static final LoggedTunableNumber IDLE_ANGLE_RAD =
      new LoggedTunableNumber("IntakePivot/IDLE_RAD", -2); // TODO: set idle angle
  public static final LoggedTunableNumber PICKUP_RAD =
      new LoggedTunableNumber("IntakePivot/PICKUP_RAD", -7); // TODO: set idle angle
  public static final LoggedTunableNumber HARDSTOP_STOW_SEEK_POSITION =
      new LoggedTunableNumber("IntakePivot/HardStop/StowSeekPosition", 10.0);
  public static final LoggedTunableNumber HARDSTOP_STOW_CURRENT_AMPS =
      new LoggedTunableNumber("IntakePivot/HardStop/StowCurrentAmps", 2.0);
  public static final LoggedTunableNumber HARDSTOP_MAX_VELOCITY_RAD_PER_SEC =
      new LoggedTunableNumber("IntakePivot/HardStop/MaxVelocityRadPerSec", 0.35);
  public static final LoggedTunableNumber HARDSTOP_SPIKE_DEBOUNCE_SEC =
      new LoggedTunableNumber("IntakePivot/HardStop/SpikeDebounceSec", 0.08);
  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_START_ANGLE_RAD = 0.0;

  private IntakePivotConstants() {}
}
