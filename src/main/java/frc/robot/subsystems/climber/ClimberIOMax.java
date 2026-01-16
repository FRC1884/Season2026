package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ClimberIOMax extends GenericElevatorSystemIOSparkMax implements ClimberIO {
  public ClimberIOMax() {
    super(new int[] {ClimberConstants.LEFT_CLIMBER, ClimberConstants.RIGHT_CLIMBER}, 40, true);

    invert(0);
  }
}
