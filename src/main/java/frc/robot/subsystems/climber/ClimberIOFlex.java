package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ClimberIOFlex extends GenericElevatorSystemIOSparkFlex implements ClimberIO {
  public ClimberIOFlex() {
    super(
        new int[] {ClimberConstants.LEFT_CLIMBER, ClimberConstants.RIGHT_CLIMBER},
        ClimberConstants.CURRENT_LIMIT_AMPS,
        ClimberConstants.BRAKE_MODE,
        ClimberConstants.POSITION_COEFFICIENT);
    if (ClimberConstants.LEFT_INVERTED) {
      invert(0);
    }
  }
}
