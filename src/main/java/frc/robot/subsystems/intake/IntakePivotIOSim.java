package frc.robot.subsystems.intake;

import frc.robot.generic.arms.GenericArmSystemIOSim;

public class IntakePivotIOSim extends GenericArmSystemIOSim implements IntakePivotIO {
  public IntakePivotIOSim() {
    super(IntakePivotConstants.SIM_MOTOR_COUNT, IntakePivotConstants.SIM_START_ANGLE_RAD);
  }
}
