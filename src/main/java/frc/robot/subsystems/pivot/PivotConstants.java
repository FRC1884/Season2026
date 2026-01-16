package frc.robot.subsystems.pivot;

import frc.robot.GlobalConstants;

public final class PivotConstants {
  public static final int PIVOT_ID = 61; // TODO: find motor id
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 1.0, REVERSE_LIMIT = 0.0; // TODO: find limits

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final double kP = 4.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains(kP, kI, kD, kS, kV, kA, 0.0);

  // TODO: tune tolerance and limits once the pivot hardware is finalized.
  public static final double POSITION_TOLERANCE = 0.0;
  public static final boolean SOFT_LIMITS_ENABLED = false;
  public static final double SOFT_LIMIT_MIN = REVERSE_LIMIT;
  public static final double SOFT_LIMIT_MAX = FORWARD_LIMIT;
  public static final double MAX_VOLTAGE = 12.0;
}
