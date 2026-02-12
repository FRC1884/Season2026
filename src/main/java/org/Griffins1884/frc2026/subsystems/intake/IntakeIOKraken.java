package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOKraken;

public class IntakeIOKraken extends GenericRollerSystemIOKraken implements IntakeIO {
  public IntakeIOKraken() {
    super(
        IntakeConstants.INTAKE_IDS,
        IntakeConstants.CURRENT_LIMIT_AMPS,
        IntakeConstants.INTAKE_INVERTED,
        IntakeConstants.BRAKE_MODE,
        IntakeConstants.REDUCTION,
        IntakeConstants.CAN_BUS);
  }
}
