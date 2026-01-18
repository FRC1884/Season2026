package frc.robot.generic.arms;

import static frc.robot.Config.Subsystems.PIVOT_ENABLED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakePivotConstants;
import frc.robot.subsystems.intake.IntakePivotIOFlex;
import frc.robot.subsystems.intake.IntakePivotIOKraken;
import frc.robot.subsystems.intake.IntakePivotIOMax;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotConstants;
import frc.robot.subsystems.shooter.ShooterPivotIOFlex;
import frc.robot.subsystems.shooter.ShooterPivotIOKraken;
import frc.robot.subsystems.shooter.ShooterPivotIOMax;
import frc.robot.subsystems.shooter.ShooterPivotIOSim;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class Arms extends SubsystemBase {
  public IntakePivotSubsystem intakePivot =
      (PIVOT_ENABLED)
          ? new IntakePivotSubsystem(
              "IntakePivot",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new IntakePivotIOSim()
                  : switch (IntakePivotConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new IntakePivotIOMax();
                    case SPARK_FLEX, VORTEX -> new IntakePivotIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new IntakePivotIOKraken();
                  })
          : null;

  public ShooterPivotSubsystem shooterPivot =
      (PIVOT_ENABLED)
          ? new ShooterPivotSubsystem(
              "ShooterPivot",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new ShooterPivotIOSim()
                  : switch (ShooterPivotConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new ShooterPivotIOMax();
                    case SPARK_FLEX, VORTEX -> new ShooterPivotIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new ShooterPivotIOKraken();
                  })
          : null;

  @Override
  public void periodic() {}
}
