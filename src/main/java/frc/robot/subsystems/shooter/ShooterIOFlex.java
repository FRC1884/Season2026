package frc.robot.subsystems.shooter;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkFlex;

public class ShooterIOFlex extends GenericRollerSystemIOSparkFlex implements ShooterIO {
  public ShooterIOFlex() {
    super(ShooterConstants.SHOOTER_ID, 40, false, true, 0.0);
  }
}
