package org.Griffins1884.frc2026.subsystems.indexer;

import com.ctre.phoenix6.CANBus;

public final class IndexerConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40,
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.KRAKEN_X60; // TODO
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INDEXER_IDS = {0}; // TODO: set indexer CAN IDs
  public static final boolean[] INDEXER_INVERTED = {false}; // TODO: set per-motor inversion

  public static final int CURRENT_LIMIT_AMPS = 40; // TODO: tune
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;
  public static final double MAX_VOLTAGE = 12.0;

  private IndexerConstants() {}
}
