package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOKraken;

public class IntakePivotIOKraken extends GenericArmSystemIOKraken implements IntakePivotIO {
  public IntakePivotIOKraken() {
    super(
        new int[] {IntakePivotConstants.PIVOT_ID},
        IntakePivotConstants.CURRENT_LIMIT_AMPS,
        IntakePivotConstants.BRAKE_MODE,
        IntakePivotConstants.FORWARD_LIMIT,
        IntakePivotConstants.REVERSE_LIMIT,
        IntakePivotConstants.POSITION_COEFFICIENT,
        IntakePivotConstants.INVERTED,
        IntakePivotConstants.CAN_BUS);
  }
}
