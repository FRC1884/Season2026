package frc.robot.generic.arms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.pivot.*;

public class Arms extends SubsystemBase {
  public PivotSubsystem pivot =
      new PivotSubsystem(
          "Pivot",
          (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
              ? new PivotIOSim(2, 0)
              : (PivotConstants.IS_FLEX) ? new PivotIOFlex() : new PivotIOMax());

  @Override
  public void periodic() {}
}
