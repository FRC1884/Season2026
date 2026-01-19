package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSim;

public class ShooterPivotIOSim extends GenericArmSystemIOSim implements ShooterPivotIO {
  public ShooterPivotIOSim() {
    super(ShooterPivotConstants.SIM_MOTOR_COUNT, ShooterPivotConstants.SIM_START_ANGLE_RAD);
  }
}
