package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOKraken;

public class ShooterPivotIOKraken extends GenericArmSystemIOKraken implements ShooterPivotIO {
  public ShooterPivotIOKraken() {
    super(
        ShooterPivotConstants.MOTOR_ID,
        ShooterPivotConstants.CURRENT_LIMIT_AMPS,
        ShooterPivotConstants.BRAKE_MODE,
        ShooterPivotConstants.FORWARD_LIMIT,
        ShooterPivotConstants.REVERSE_LIMIT,
        ShooterPivotConstants.POSITION_COEFFICIENT,
        ShooterPivotConstants.INVERTED,
        ShooterPivotConstants.CAN_BUS);
  }
}
