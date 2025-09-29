package frc.robot.generic.elevators;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.exampleClasses.elevator.ElevatorIOFlex;
import frc.robot.subsystems.exampleClasses.elevator.ElevatorSubsystem;
import frc.robot.subsystems.exampleClasses.elevator.ElevatorSubsystem.ElevatorGoal;

public class Elevators extends SubsystemBase {
  private ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem("Elevator", new ElevatorIOFlex());

  @Override
  public void periodic() {
    elevatorSubsystem.periodic();
  }

  public void testing() {
    elevatorSubsystem.setGoal(ElevatorGoal.TESTING);
  }

  public void idle() {
    elevatorSubsystem.setGoal(ElevatorGoal.IDLING);
  }
}
