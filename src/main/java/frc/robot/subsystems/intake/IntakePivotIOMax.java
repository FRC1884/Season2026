package frc.robot.subsystems.intake;

import frc.robot.generic.arms.GenericArmSystemIOSparkMax;

public class IntakePivotIOMax extends GenericArmSystemIOSparkMax implements IntakePivotIO {
  public IntakePivotIOMax() {
    super(
        new int[] {IntakePivotConstants.PIVOT_ID},
        IntakePivotConstants.CURRENT_LIMIT_AMPS,
        IntakePivotConstants.BRAKE_MODE,
        IntakePivotConstants.FORWARD_LIMIT,
        IntakePivotConstants.REVERSE_LIMIT,
        IntakePivotConstants.POSITION_COEFFICIENT);
    if (IntakePivotConstants.INVERTED) {
      invert(0);
    }
  }
}
