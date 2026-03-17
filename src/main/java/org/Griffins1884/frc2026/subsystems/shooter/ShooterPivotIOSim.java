package org.Griffins1884.frc2026.subsystems.shooter;

import org.Griffins1884.frc2026.mechanisms.arms.MechanismArmIOSim;

public class ShooterPivotIOSim extends MechanismArmIOSim implements ShooterPivotIO {
  public ShooterPivotIOSim() {
    super(ShooterPivotConstants.SIM_MOTOR_COUNT, ShooterPivotConstants.SIM_START_ANGLE_RAD);
  }
}
