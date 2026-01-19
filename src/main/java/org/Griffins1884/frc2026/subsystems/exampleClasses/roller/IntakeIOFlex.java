package org.Griffins1884.frc2026.subsystems.exampleClasses.roller;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  public IntakeIOFlex() {
    super(IntakeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
