package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSparkMax;

public class IntakePivotIOMax extends GenericArmSystemIOSparkMax implements IntakePivotIO {
  public IntakePivotIOMax() {
    super(
        IntakePivotConstants.MOTOR_ID,
        IntakePivotConstants.CURRENT_LIMIT_AMPS,
        IntakePivotConstants.BRAKE_MODE,
        IntakePivotConstants.FORWARD_LIMIT,
        IntakePivotConstants.REVERSE_LIMIT,
        IntakePivotConstants.POSITION_COEFFICIENT);
    boolean[] inverted = IntakePivotConstants.INVERTED;
    for (int i = 0; i < inverted.length; i++) {
      if (inverted[i]) {
        invert(i);
      }
    }
  }
}
