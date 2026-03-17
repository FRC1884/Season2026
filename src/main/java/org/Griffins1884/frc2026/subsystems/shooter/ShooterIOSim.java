package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.mechanisms.rollers.MechanismRollerIOSim;

public class ShooterIOSim extends MechanismRollerIOSim implements ShooterIO {
  public ShooterIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
