// change between sim and sparkmax files

package org.Griffins1884.frc2026.subsystems.exampleClasses.elevator;

import org.Griffins1884.frc2026.generic.elevators.GenericElevatorSystemIOSim;

public class ElevatorIOSim extends GenericElevatorSystemIOSim implements ElevatorIO {
  public ElevatorIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
