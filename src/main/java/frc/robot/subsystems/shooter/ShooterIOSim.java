package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class ShooterIOSim extends GenericRollerSystemIOSim implements ShooterIO {
  public ShooterIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
