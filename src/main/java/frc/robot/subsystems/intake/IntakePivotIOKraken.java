package frc.robot.subsystems.intake;

import frc.robot.generic.arms.GenericArmSystemIOKraken;

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
