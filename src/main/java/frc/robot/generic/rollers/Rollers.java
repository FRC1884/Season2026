package frc.robot.generic.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOFlex;
import frc.robot.subsystems.intake.IntakeIOMax;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOFlex;
import frc.robot.subsystems.shooter.ShooterIOMax;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Rollers extends SubsystemBase {
  public IntakeSubsystem intake =
      new IntakeSubsystem(
          "Intake",
          (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
              ? new IntakeIOSim(DCMotor.getNeoVortex(2), 1, 1)
              : (IntakeConstants.isFlex) ? new IntakeIOFlex() : new IntakeIOMax());
  public ShooterSubsystem shooter =
      new ShooterSubsystem(
          "Shooter",
          (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
              ? new ShooterIOSim(DCMotor.getNeoVortex(2), 1, 1)
              : (IntakeConstants.isFlex) ? new ShooterIOFlex() : new ShooterIOMax());

  @Override
  public void periodic() {}
}
