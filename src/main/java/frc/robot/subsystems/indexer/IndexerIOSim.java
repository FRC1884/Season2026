package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class IndexerIOSim extends GenericRollerSystemIOSim implements IndexerIO {
  public IndexerIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
