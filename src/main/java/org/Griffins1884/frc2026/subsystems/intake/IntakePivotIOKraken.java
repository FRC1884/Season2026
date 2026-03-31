package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.mechanisms.arms.MechanismArmIOKraken;

public class IntakePivotIOKraken extends MechanismArmIOKraken implements IntakePivotIO {
  public IntakePivotIOKraken(int id, boolean inverted) {
    super(
        new int[] {id},
        IntakePivotConstants.CURRENT_LIMIT_AMPS,
        IntakePivotConstants.BRAKE_MODE,
        IntakePivotConstants.FORWARD_LIMIT,
        IntakePivotConstants.REVERSE_LIMIT,
        IntakePivotConstants.POSITION_COEFFICIENT,
        new boolean[] {inverted},
        IntakePivotConstants.CAN_BUS,
        IntakePivotConstants.MOTION_MAGIC_CRUISE_VEL.get(),
        IntakePivotConstants.MOTION_MAGIC_ACCEL.get(),
        IntakePivotConstants.MOTION_MAGIC_JERK.get(),
        IntakePivotConstants.KRAKEN_FEATURES);
  }
}
