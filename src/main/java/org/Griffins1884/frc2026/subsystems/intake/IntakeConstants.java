package org.Griffins1884.frc2026.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;

public final class IntakeConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40,
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.KRAKEN_X60;
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INTAKE_IDS = {62}; // TODO: set intake CAN IDs
  public static final boolean[] INTAKE_INVERTED = {false}; // TODO: set per-motor inversion
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 0.0;

  public static final GlobalConstants.Gains gains =
      new GlobalConstants.Gains("Intake/Gains", 1.0, 0.0, 0.0);
  public static final double VELOCITY_TOLERANCE = 0.0; // TODO: tune for intake
  public static final double MAX_VOLTAGE = 12.0; // TODO: tune for intake

  private IntakeConstants() {}
}
