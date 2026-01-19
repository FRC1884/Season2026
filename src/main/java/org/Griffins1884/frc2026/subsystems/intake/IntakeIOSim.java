package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSim;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
  public IntakeIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
