package org.Griffins1884.frc2026.subsystems.exampleClasses.shooter;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSparkFlex;

public class ShooterIOFlex extends GenericRollerSystemIOSparkFlex implements ShooterIO {
  public ShooterIOFlex() {
    super(ShooterConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
