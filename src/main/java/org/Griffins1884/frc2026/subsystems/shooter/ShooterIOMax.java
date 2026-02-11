package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSparkMax;

public class ShooterIOMax extends GenericRollerSystemIOSparkMax implements ShooterIO {
  public ShooterIOMax() {
    super(
        ShooterConstants.SHOOTER_IDS[0],
        ShooterConstants.CURRENT_LIMIT_AMPS,
        ShooterConstants.SHOOTER_INVERTED[0],
        ShooterConstants.BRAKE_MODE,
        ShooterConstants.REDUCTION);
  }
}
