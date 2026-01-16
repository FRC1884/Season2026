package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.turrets.GenericTurretSystemIOSim;

public class TurretIOSim extends GenericTurretSystemIOSim implements TurretIO {
  public TurretIOSim() {
    super(
        DCMotor.getNeoVortex(TurretConstants.SIM_MOTOR_COUNT),
        TurretConstants.GEAR_RATIO,
        TurretConstants.SIM_MOI);
  }
}
