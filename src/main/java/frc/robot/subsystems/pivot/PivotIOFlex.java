package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.pivot.PivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arms.GenericArmSystemIOSparkFlex;

public class PivotIOFlex extends GenericArmSystemIOSparkFlex implements PivotIO {

  public PivotIOFlex() {
    super(new int[] {PivotConstants.PIVOT_ID}, 40, true, FORWARD_LIMIT, REVERSE_LIMIT);
    invert(0);
  }
}
