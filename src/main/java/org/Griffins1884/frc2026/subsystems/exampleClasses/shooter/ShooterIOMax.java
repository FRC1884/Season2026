package org.Griffins1884.frc2026.subsystems.exampleClasses.shooter;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSparkMax;

public class ShooterIOMax extends GenericRollerSystemIOSparkMax implements ShooterIO {
  public ShooterIOMax() {
    super(ShooterConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
