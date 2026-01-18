package frc.robot.subsystems.shooter;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class ShooterIOMax extends GenericRollerSystemIOSparkMax implements ShooterIO {
  public ShooterIOMax() {
    super(ShooterConstants.SHOOTER_ID, 40, false, true, 0.0);
  }
}
