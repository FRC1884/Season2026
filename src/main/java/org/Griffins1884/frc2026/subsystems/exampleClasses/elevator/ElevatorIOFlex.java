package org.Griffins1884.frc2026.subsystems.exampleClasses.elevator;

import org.Griffins1884.frc2026.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ElevatorIOFlex extends GenericElevatorSystemIOSparkFlex implements ElevatorIO {
  public ElevatorIOFlex() {
    super(new int[] {ElevatorConstants.LEFT_ELEVATOR, ElevatorConstants.RIGHT_ELEVATOR}, 40, true);

    invert(0);
  }
}
