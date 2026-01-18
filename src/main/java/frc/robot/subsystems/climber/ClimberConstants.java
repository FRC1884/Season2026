package frc.robot.subsystems.climber;

import frc.robot.GlobalConstants;

public final class ClimberConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40,
    VORTEX
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.SPARK_FLEX; // TODO

  public static final int LEFT_CLIMBER = 42; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_CLIMBER = 41; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = false;

  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;

  public static final double PULLEY_RADIUS_METERS = 0.016; // TODO: verify pulley radius
  public static final double GEAR_RATIO = 1.0; // TODO: set reduction
  public static final double POSITION_COEFFICIENT =
      (2.0 * Math.PI * PULLEY_RADIUS_METERS) / GEAR_RATIO;

  public static final GlobalConstants.Gains GAINS = new GlobalConstants.Gains(0.7, 0.0, 0.0);
  public static final double POSITION_TOLERANCE = 0.01; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = false;
  public static final double SOFT_LIMIT_MIN = 0.0;
  public static final double SOFT_LIMIT_MAX = 1.0; // TODO: set max extension
  public static final double MAX_VOLTAGE = 12.0;
  public static final double STOW_HEIGHT_METERS = 0.0; // TODO: set stow height

  public static final int LIMIT_SWITCH_DIO = 9; // TODO: wire limit switch if used

  public static final int SIM_MOTOR_COUNT = 2;
  public static final double SIM_START_HEIGHT_METERS = 0.0;

  private ClimberConstants() {}
}
