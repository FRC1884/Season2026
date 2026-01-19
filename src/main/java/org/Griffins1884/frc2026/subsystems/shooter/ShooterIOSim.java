package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSim;

public class ShooterIOSim extends GenericRollerSystemIOSim implements ShooterIO {
  public ShooterIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
