package org.Griffins1884.frc2026.generic.arms;

import static org.Griffins1884.frc2026.Config.Subsystems.INTAKE_PIVOT_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.SHOOTER_PIVOT_ENABLED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotIOFlex;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotIOKraken;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotIOMax;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotIOSim;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotSubsystem;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotIOFlex;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotIOKraken;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotIOMax;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotIOSim;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem;

public class Arms extends SubsystemBase {
  public IntakePivotSubsystem intakePivot =
      (INTAKE_PIVOT_ENABLED)
          ? new IntakePivotSubsystem(
              "IntakePivot",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new IntakePivotIOSim()
                  : switch (IntakePivotConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new IntakePivotIOMax();
                    case SPARK_FLEX -> new IntakePivotIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new IntakePivotIOKraken();
                  })
          : null;

  public ShooterPivotSubsystem shooterPivot =
      (SHOOTER_PIVOT_ENABLED)
          ? new ShooterPivotSubsystem(
              "ShooterPivot",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new ShooterPivotIOSim()
                  : switch (ShooterPivotConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new ShooterPivotIOMax();
                    case SPARK_FLEX -> new ShooterPivotIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new ShooterPivotIOKraken();
                  })
          : null;

}
