package org.Griffins1884.frc2026.subsystems.turret;

import org.Griffins1884.frc2026.generic.turrets.GenericPositionTurretSystem;

public class TurretSubsystem extends GenericPositionTurretSystem {
  public TurretSubsystem(TurretIO io) {
    super(
        "Turret",
        io,
        new TurretConfig(
            TurretConstants.GAINS,
            TurretConstants.POSITION_TOLERANCE_RAD,
            TurretConstants.MAX_VELOCITY_RAD_PER_SEC,
            TurretConstants.MAX_ACCEL_RAD_PER_SEC2,
            TurretConstants.SOFT_LIMITS_ENABLED,
            TurretConstants.SOFT_LIMIT_MIN_RAD,
            TurretConstants.SOFT_LIMIT_MAX_RAD,
            TurretConstants.USE_ABSOLUTE_ENCODER,
            TurretConstants.ABSOLUTE_ENCODER_OFFSET_RAD,
            TurretConstants.MAX_VOLTAGE));
    if (TurretConstants.CONTINUOUS_INPUT) {
      enableContinuousInput(-Math.PI, Math.PI);
    }
  }
}
