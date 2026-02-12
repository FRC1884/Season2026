package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  public IntakeIOFlex() {
    super(
        IntakeConstants.INTAKE_IDS[0],
        IntakeConstants.CURRENT_LIMIT_AMPS,
        IntakeConstants.INTAKE_INVERTED[0],
        IntakeConstants.BRAKE_MODE,
        IntakeConstants.REDUCTION);
  }
}
