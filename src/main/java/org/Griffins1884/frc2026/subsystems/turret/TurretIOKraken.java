package org.Griffins1884.frc2026.subsystems.turret;

import org.Griffins1884.frc2026.generic.turrets.GenericTurretSystemIOKraken;

public class TurretIOKraken extends GenericTurretSystemIOKraken implements TurretIO {
  public TurretIOKraken() {
    super(
        TurretConstants.TURRET_ID,
        TurretConstants.CURRENT_LIMIT_AMPS,
        TurretConstants.INVERTED,
        TurretConstants.BRAKE_MODE,
        TurretConstants.GEAR_RATIO,
        TurretConstants.CANCODER_ID,
        TurretConstants.CANCODER_INVERTED,
        TurretConstants.ABSOLUTE_ENCODER_GEAR_RATIO);
  }
}
