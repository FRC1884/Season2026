package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSim;

public class ClimberIOSim extends GenericElevatorSystemIOSim implements ClimberIO {
  public ClimberIOSim() {
    super(ClimberConstants.SIM_MOTOR_COUNT, ClimberConstants.SIM_START_HEIGHT_METERS);
  }
}
