package org.Griffins1884.frc2026.subsystems.objectivetracker;

import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/** Queue-autonomous and REBUILT gameplay tuning values exposed for dashboard tuning. */
public final class RebuiltAutoConstants {
  public static final LoggedTunableNumber QUEUE_ALIGN_TIMEOUT_SEC =
      new LoggedTunableNumber("RebuiltAuto/QueueAlignTimeoutSec", 4.5);
  public static final LoggedTunableNumber QUEUE_ALIGN_CONSTRAINT_FACTOR =
      new LoggedTunableNumber("RebuiltAuto/QueueAlignConstraintFactor", 0.7);
  public static final LoggedTunableNumber QUEUE_ALIGN_TOLERANCE_METERS =
      new LoggedTunableNumber("RebuiltAuto/QueueAlignToleranceMeters", 0.08);
  public static final LoggedTunableNumber QUEUE_FLOW_THROUGH_END_VELOCITY_MPS =
      new LoggedTunableNumber("RebuiltAuto/QueueFlowThroughEndVelocityMps", 1.0);

  public static final LoggedTunableNumber SHOOT_WHILE_MOVING_MAX_SPEED_MPS =
      new LoggedTunableNumber("RebuiltAuto/ShootWhileMovingMaxSpeedMps", 3.0);
  public static final LoggedTunableNumber INTAKE_WHILE_MOVING_MAX_SPEED_MPS =
      new LoggedTunableNumber("RebuiltAuto/IntakeWhileMovingMaxSpeedMps", 4.0);

  private RebuiltAutoConstants() {}
}
