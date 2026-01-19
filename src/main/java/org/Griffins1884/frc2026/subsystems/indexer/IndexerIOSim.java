package org.Griffins1884.frc2026.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOSim;

public class IndexerIOSim extends GenericRollerSystemIOSim implements IndexerIO {
  public IndexerIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
