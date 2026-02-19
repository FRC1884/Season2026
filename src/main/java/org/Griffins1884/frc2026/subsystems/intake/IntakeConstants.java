package org.Griffins1884.frc2026.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class IntakeConstants {
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INTAKE_IDS = {21}; // TODO: set intake CAN IDs
  public static final boolean[] INTAKE_INVERTED = {false}; // TODO: set per-motor inversion
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0; // TODO: set gear ratio

  public static final GlobalConstants.Gains gains =
      new GlobalConstants.Gains("Intake/Gains", 0.002, 0.02, 1.0, 0.32726, 0.017, 0.15908);
  public static final double VELOCITY_TOLERANCE = 0.0; // TODO: tune for intake
  public static final double MAX_VOLTAGE = 12.0; // TODO: tune for intake
  public static final LoggedTunableNumber FORWARD_RPM =
      new LoggedTunableNumber("Intake/ForwardRpm", 2500.0);
  public static final LoggedTunableNumber REVERSE_RPM =
      new LoggedTunableNumber("Intake/ReverseRpm", -2500.0);

  private IntakeConstants() {}
}
