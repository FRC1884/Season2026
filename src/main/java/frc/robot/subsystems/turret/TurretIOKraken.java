package frc.robot.subsystems.turret;

import frc.robot.generic.turrets.GenericTurretSystemIOKraken;

public class TurretIOKraken extends GenericTurretSystemIOKraken implements TurretIO {
  public TurretIOKraken() {
    super(
        TurretConstants.MOTOR_ID,
        TurretConstants.CURRENT_LIMIT_AMPS,
        TurretConstants.INVERTED,
        TurretConstants.BRAKE_MODE,
        TurretConstants.GEAR_RATIO,
        TurretConstants.CANCODER_ID,
        TurretConstants.CANCODER_INVERTED,
        TurretConstants.ABSOLUTE_ENCODER_GEAR_RATIO,
        TurretConstants.CAN_BUS);
  }
}
