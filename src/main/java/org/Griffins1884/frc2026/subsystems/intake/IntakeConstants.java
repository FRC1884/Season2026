package org.Griffins1884.frc2026.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class IntakeConstants {
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INTAKE_IDS = {21};
  public static final boolean[] INTAKE_INVERTED = {false};
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final MechanismDefinition.KrakenFeatureConfig KRAKEN_FEATURES =
      new MechanismDefinition.KrakenFeatureConfig(true, true, false, 100, true);
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;

  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("Intake/Gains", 0.002, 0.02, 1.0, 0.32726, 0.017, 0.15908);
  public static final double VELOCITY_TOLERANCE = 0.0;
  public static final double MAX_VOLTAGE = 12.0;
  public static final LoggedTunableNumber FORWARD_RPM =
      new LoggedTunableNumber("Intake/ForwardRpm", 12.0);
  public static final LoggedTunableNumber REVERSE_RPM =
      new LoggedTunableNumber("Intake/ReverseRpm", -8.0);

  private IntakeConstants() {}
}
