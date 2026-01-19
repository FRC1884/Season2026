package org.Griffins1884.frc2026.subsystems.exampleClasses.pivot;

import static org.Griffins1884.frc2026.subsystems.exampleClasses.pivot.PivotConstants.FORWARD_LIMIT;
import static org.Griffins1884.frc2026.subsystems.exampleClasses.pivot.PivotConstants.REVERSE_LIMIT;

import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIOSparkFlex;

public class PivotIOFlex extends GenericArmSystemIOSparkFlex implements PivotIO {

  public PivotIOFlex() {
    super(new int[] {PivotConstants.PIVOT_ID}, 40, true, FORWARD_LIMIT, REVERSE_LIMIT);
    invert(0);
  }
}
