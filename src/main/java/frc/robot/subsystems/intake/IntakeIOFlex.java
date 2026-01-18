package frc.robot.subsystems.intake;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  public IntakeIOFlex() {
    super(IntakeConstants.INTAKE_ID, 40, false, true, 0.0);
  }
}
