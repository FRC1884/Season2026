package frc.robot.subsystems.exampleClasses.shooter;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkFlex;

public class ShooterIOFlex extends GenericRollerSystemIOSparkFlex implements ShooterIO {
  public ShooterIOFlex() {
    super(ShooterConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
