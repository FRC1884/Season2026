package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSim;

public class IntakePivotIOSim extends GenericArmSystemIOSim implements IntakePivotIO {
  public IntakePivotIOSim() {
    super(IntakePivotConstants.SIM_MOTOR_COUNT, IntakePivotConstants.SIM_START_ANGLE_RAD);
  }
}
