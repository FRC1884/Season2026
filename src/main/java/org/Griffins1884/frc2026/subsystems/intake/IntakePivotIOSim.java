package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.mechanisms.arms.MechanismArmIOSim;

public class IntakePivotIOSim extends MechanismArmIOSim implements IntakePivotIO {
  public IntakePivotIOSim() {
    super(IntakePivotConstants.SIM_MOTOR_COUNT, IntakePivotConstants.SIM_START_ANGLE_RAD);
  }
}
