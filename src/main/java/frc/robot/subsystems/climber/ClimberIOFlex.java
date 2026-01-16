package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ClimberIOFlex extends GenericElevatorSystemIOSparkFlex implements ClimberIO {
  public ClimberIOFlex() {
    super(new int[] {ClimberConstants.LEFT_CLIMBER, ClimberConstants.RIGHT_CLIMBER}, 40, true);

    invert(0);
  }
}
