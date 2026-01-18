package frc.robot.subsystems.turret;

import frc.robot.generic.turrets.GenericTurretSystemIOSparkMax;

public class TurretIOSparkMax extends GenericTurretSystemIOSparkMax implements TurretIO {
  public TurretIOSparkMax() {
    super(
        TurretConstants.TURRET_ID,
        TurretConstants.CURRENT_LIMIT_AMPS,
        TurretConstants.INVERTED,
        TurretConstants.BRAKE_MODE,
        TurretConstants.GEAR_RATIO,
        TurretConstants.ABSOLUTE_ENCODER_GEAR_RATIO);
  }
}
