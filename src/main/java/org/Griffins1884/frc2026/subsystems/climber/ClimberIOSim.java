package org.Griffins1884.frc2026.subsystems.climber;

import org.Griffins1884.frc2026.generic.elevators.GenericElevatorSystemIOSim;

public class ClimberIOSim extends GenericElevatorSystemIOSim implements ClimberIO {
  public ClimberIOSim() {
    super(ClimberConstants.SIM_MOTOR_COUNT, ClimberConstants.SIM_START_HEIGHT_METERS);
  }
}
