package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOKraken;

public class ShooterIOKraken extends GenericRollerSystemIOKraken implements ShooterIO {
  public ShooterIOKraken() {
    super(
        ShooterConstants.SHOOTER_IDS,
        ShooterConstants.CURRENT_LIMIT_AMPS,
        ShooterConstants.SHOOTER_INVERTED,
        ShooterConstants.BRAKE_MODE,
        ShooterConstants.REDUCTION,
        ShooterConstants.CAN_BUS);
  }
}
