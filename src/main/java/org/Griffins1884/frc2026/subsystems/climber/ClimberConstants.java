package org.Griffins1884.frc2026.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;

public final class ClimberConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.KRAKEN_X60; // TODO
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] CLIMBER_IDS = {42}; // TODO: Change to correct Motor ID's
  public static final boolean[] CLIMBER_INVERTED = {true};

  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;

  public static final double PULLEY_RADIUS_METERS = 0.016; // TODO: verify pulley radius
  public static final double GEAR_RATIO = 1.0; // TODO: set reduction
  public static final double POSITION_COEFFICIENT =
      (2.0 * Math.PI * PULLEY_RADIUS_METERS) / GEAR_RATIO;

  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("Climber/Gains", 0.0, 0.0, 0.0); // TODO: tune
  public static final double POSITION_TOLERANCE = 0.01; // TODO: tune
  public static final boolean SOFT_LIMITS_ENABLED = false;
  public static final double SOFT_LIMIT_MIN = 0.0;
  public static final double SOFT_LIMIT_MAX = 1.0; // TODO: set max extension
  public static final double MAX_VOLTAGE = 12.0;
  public static final double STOW_HEIGHT_METERS = 0.0; // TODO: set stow height

  public static final int SIM_MOTOR_COUNT = 2;
  public static final double SIM_START_HEIGHT_METERS = 0.0;

  private ClimberConstants() {}
}
