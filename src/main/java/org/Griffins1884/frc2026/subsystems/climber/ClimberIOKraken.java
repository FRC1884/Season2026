package org.Griffins1884.frc2026.subsystems.climber;

import org.Griffins1884.frc2026.generic.elevators.GenericElevatorSystemIOKraken;

public class ClimberIOKraken extends GenericElevatorSystemIOKraken implements ClimberIO {
  public ClimberIOKraken() {
    super(
        new int[] {ClimberConstants.LEFT_CLIMBER, ClimberConstants.RIGHT_CLIMBER},
        ClimberConstants.CURRENT_LIMIT_AMPS,
        ClimberConstants.BRAKE_MODE,
        ClimberConstants.POSITION_COEFFICIENT);
    if (ClimberConstants.LEFT_INVERTED) {
      invert(0);
    }
    if (ClimberConstants.RIGHT_INVERTED) {
      invert(1);
    }
  }
}
