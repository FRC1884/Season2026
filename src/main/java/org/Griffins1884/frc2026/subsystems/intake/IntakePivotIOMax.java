package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSparkMax;

public class IntakePivotIOMax extends GenericArmSystemIOSparkMax implements IntakePivotIO {
  public IntakePivotIOMax(int id, boolean inverted) {
    super(
        new int[] {id},
        IntakePivotConstants.CURRENT_LIMIT_AMPS,
        IntakePivotConstants.BRAKE_MODE,
        IntakePivotConstants.FORWARD_LIMIT,
        IntakePivotConstants.REVERSE_LIMIT,
        IntakePivotConstants.POSITION_COEFFICIENT);
    if (inverted) {
      invert(0);
    }
  }
}
