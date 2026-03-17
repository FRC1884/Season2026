package org.Griffins1884.frc2026.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import org.Griffins1884.frc2026.mechanisms.turrets.MechanismTurretIOSim;

public class TurretIOSim extends MechanismTurretIOSim implements TurretIO {
  public TurretIOSim() {
    super(
        DCMotor.getNeoVortex(TurretConstants.SIM_MOTOR_COUNT),
        TurretConstants.GEAR_RATIO,
        TurretConstants.SIM_MOI,
        TurretConstants.INVERTED);
  }
}
