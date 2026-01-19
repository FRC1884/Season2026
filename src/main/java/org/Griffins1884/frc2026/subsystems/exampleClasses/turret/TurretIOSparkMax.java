package org.Griffins1884.frc2026.subsystems.exampleClasses.turret;

import org.Griffins1884.frc2026.generic.turrets.GenericTurretSystemIOSparkMax;

public class TurretIOSparkMax extends GenericTurretSystemIOSparkMax implements TurretIO {
  public TurretIOSparkMax() {
    super(
        TurretConstants.MOTOR_ID,
        TurretConstants.CURRENT_LIMIT_AMPS,
        TurretConstants.INVERTED,
        TurretConstants.BRAKE_MODE,
        TurretConstants.GEAR_RATIO,
        TurretConstants.ABSOLUTE_ENCODER_GEAR_RATIO);
  }
}
