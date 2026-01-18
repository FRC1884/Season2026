package frc.robot.subsystems.shooter;

import frc.robot.generic.arms.GenericArmSystemIOSim;

public class ShooterPivotIOSim extends GenericArmSystemIOSim implements ShooterPivotIO {
  public ShooterPivotIOSim() {
    super(ShooterPivotConstants.SIM_MOTOR_COUNT, ShooterPivotConstants.SIM_START_ANGLE_RAD);
  }
}
