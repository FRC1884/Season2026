package frc.robot.generic.arms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.pivot.*;

import static frc.robot.Config.Subsystems.PIVOT_ENABLED;

public class Arms extends SubsystemBase {
  public PivotSubsystem pivot = (PIVOT_ENABLED) ?
      new PivotSubsystem(
          "Pivot",
          (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
              ? new PivotIOSim(2, 0)
              : (PivotConstants.IS_FLEX) ? new PivotIOFlex() : new PivotIOMax())
          : null;

  @Override
  public void periodic() {}
}
