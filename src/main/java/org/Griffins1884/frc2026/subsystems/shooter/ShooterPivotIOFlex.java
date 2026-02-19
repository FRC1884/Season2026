package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSparkFlex;

public class ShooterPivotIOFlex extends GenericArmSystemIOSparkFlex implements ShooterPivotIO {
  public ShooterPivotIOFlex() {
    super(
        ShooterPivotConstants.MOTOR_ID,
        ShooterPivotConstants.CURRENT_LIMIT_AMPS,
        ShooterPivotConstants.BRAKE_MODE,
        ShooterPivotConstants.FORWARD_LIMIT,
        ShooterPivotConstants.REVERSE_LIMIT,
        ShooterPivotConstants.POSITION_COEFFICIENT);
    boolean[] inverted = ShooterPivotConstants.INVERTED;
    for (int i = 0; i < inverted.length; i++) {
      if (inverted[i]) {
        invert(i);
      }
    }
  }
}
