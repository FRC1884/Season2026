package frc.robot.subsystems.exampleClasses.shooter;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class ShooterIOMax extends GenericRollerSystemIOSparkMax implements ShooterIO {
  public ShooterIOMax() {
    super(ShooterConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
