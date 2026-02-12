package org.Griffins1884.frc2026.subsystems.climber;

import org.Griffins1884.frc2026.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ClimberIOMax extends GenericElevatorSystemIOSparkMax implements ClimberIO {
  public ClimberIOMax() {
    super(
        ClimberConstants.CLIMBER_IDS,
        ClimberConstants.CURRENT_LIMIT_AMPS,
        ClimberConstants.BRAKE_MODE,
        ClimberConstants.POSITION_COEFFICIENT);
    boolean[] inverted = ClimberConstants.CLIMBER_INVERTED;
    for (int i = 0; i < inverted.length; i++) {
      if (inverted[i]) {
        invert(i);
      }
    }
  }
}
