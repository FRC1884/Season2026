package org.Griffins1884.frc2026.subsystems.exampleClasses.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.generic.turrets.GenericTurretSystemIOSim;

public class TurretIOSim extends GenericTurretSystemIOSim implements TurretIO {
  public TurretIOSim() {
    super(
        DCMotor.getNeoVortex(TurretConstants.SIM_MOTOR_COUNT),
        TurretConstants.GEAR_RATIO,
        TurretConstants.SIM_MOI);
  }
}
