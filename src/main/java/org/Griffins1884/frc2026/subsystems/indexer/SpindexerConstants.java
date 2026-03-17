package org.Griffins1884.frc2026.subsystems.indexer;

import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class SpindexerConstants {
  public static final MechanismDefinition.MotorControllerType MOTOR_CONTROLLER =
      MechanismDefinition.MotorControllerType.SIMULATION_ONLY;
  public static final String CAN_BUS = "";
  public static final int MOTOR_ID = 91;
  public static final boolean INVERTED = false;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;
  public static final double MAX_VOLTAGE = 12.0;
  public static final double VELOCITY_TOLERANCE = 75.0;

  public static final LoggedTunableNumber INDEX_RPM =
      new LoggedTunableNumber("Spindexer/IndexRpm", 250.0);
  public static final LoggedTunableNumber REVERSE_RPM =
      new LoggedTunableNumber("Spindexer/ReverseRpm", -180.0);
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("Spindexer/Gains", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private SpindexerConstants() {}
}
