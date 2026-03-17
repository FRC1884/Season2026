package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class ToothRolloutConstants {
  public static final MechanismDefinition.MotorControllerType MOTOR_CONTROLLER =
      MechanismDefinition.MotorControllerType.SIMULATION_ONLY;
  public static final String CAN_BUS = "";
  public static final int MOTOR_ID = 90;
  public static final boolean INVERTED = false;
  public static final int CURRENT_LIMIT_AMPS = 30;
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;
  public static final double MAX_VOLTAGE = 12.0;

  public static final LoggedTunableNumber DEPLOY_VOLTS =
      new LoggedTunableNumber("ToothRollout/DeployVolts", 8.0);
  public static final LoggedTunableNumber RETRACT_VOLTS =
      new LoggedTunableNumber("ToothRollout/RetractVolts", -6.0);
  public static final LoggedTunableNumber HOLD_VOLTS =
      new LoggedTunableNumber("ToothRollout/HoldVolts", 1.0);
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("ToothRollout/Gains", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private ToothRolloutConstants() {}
}
