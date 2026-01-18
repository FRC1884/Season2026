package frc.robot.subsystems.shooter;

import frc.robot.generic.arms.GenericArmSystemIOSparkMax;

public class ShooterPivotIOMax extends GenericArmSystemIOSparkMax implements ShooterPivotIO {
  public ShooterPivotIOMax() {
    super(
        new int[] {ShooterPivotConstants.MOTOR_ID},
        ShooterPivotConstants.CURRENT_LIMIT_AMPS,
        ShooterPivotConstants.BRAKE_MODE,
        ShooterPivotConstants.FORWARD_LIMIT,
        ShooterPivotConstants.REVERSE_LIMIT,
        ShooterPivotConstants.POSITION_COEFFICIENT);
    if (ShooterPivotConstants.INVERTED) {
      invert(0);
    }
  }
}
