package frc.robot.generic.elevators;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIOFlex;
import frc.robot.subsystems.climber.ClimberIOMax;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class Elevators extends SubsystemBase {
  public ClimberSubsystem climber =
      new ClimberSubsystem(
          "Elevator",
          (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
              ? new ClimberIOSim()
              : (ClimberConstants.isFlex) ? new ClimberIOFlex() : new ClimberIOMax(),
          new ClimberSubsystem.ClimberConfig(ClimberConstants.MAX_VOLTAGE));

  @Override
  public void periodic() {}
}
