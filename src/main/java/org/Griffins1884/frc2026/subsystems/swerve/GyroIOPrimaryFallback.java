package org.Griffins1884.frc2026.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Gyro adapter that uses a primary source (Pigeon) and falls back to a secondary source (NavX) when
 * primary connectivity is lost.
 */
public class GyroIOPrimaryFallback implements GyroIO {
  private static final int PRIMARY_FAILOVER_CYCLES = 3;
  private static final int PRIMARY_REACQUIRE_CYCLES = 10;
  private static final double PRIMARY_OFFSET_DECAY_PER_CYCLE_DEG = 1.0;

  private final GyroIO primary;
  private final GyroIO secondary;
  private final GyroIOInputs primaryInputs = new GyroIOInputs();
  private final GyroIOInputs secondaryInputs = new GyroIOInputs();

  private boolean preferSecondary = false;
  private int primaryUnhealthyCycles = 0;
  private int primaryHealthyCycles = 0;
  private Rotation2d primaryOffset = new Rotation2d();
  private Rotation2d secondaryOffset = new Rotation2d();
  private Rotation2d lastPublishedYaw = new Rotation2d();
  private boolean hasPublishedYaw = false;

  public GyroIOPrimaryFallback(GyroIO primary, GyroIO secondary) {
    this.primary = primary;
    this.secondary = secondary;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    primary.updateInputs(primaryInputs);
    secondary.updateInputs(secondaryInputs);

    boolean primaryConnected = primaryInputs.connected;
    boolean secondaryConnected = secondaryInputs.connected;
    if (primaryConnected) {
      primaryUnhealthyCycles = 0;
    } else {
      primaryUnhealthyCycles++;
    }

    if (preferSecondary) {
      if (secondaryConnected) {
        if (primaryConnected) {
          primaryHealthyCycles++;
          if (primaryHealthyCycles >= PRIMARY_REACQUIRE_CYCLES) {
            Rotation2d referenceYaw =
                hasPublishedYaw ? lastPublishedYaw : secondaryInputs.yawPosition;
            primaryOffset = referenceYaw.minus(primaryInputs.yawPosition);
            preferSecondary = false;
            primaryHealthyCycles = 0;
          }
        } else {
          primaryHealthyCycles = 0;
        }
      } else {
        preferSecondary = false;
        primaryHealthyCycles = 0;
      }
    }

    if (!preferSecondary
        && !primaryConnected
        && secondaryConnected
        && primaryUnhealthyCycles >= PRIMARY_FAILOVER_CYCLES) {
      preferSecondary = true;
      primaryHealthyCycles = 0;
      Rotation2d referenceYaw = hasPublishedYaw ? lastPublishedYaw : primaryInputs.yawPosition;
      secondaryOffset = referenceYaw.minus(secondaryInputs.yawPosition);
    }

    boolean useSecondary = preferSecondary && secondaryConnected;
    GyroIOInputs activeInputs = useSecondary ? secondaryInputs : primaryInputs;

    inputs.connected = primaryConnected || secondaryConnected;
    inputs.primaryConnected = primaryConnected;
    inputs.secondaryConnected = secondaryConnected;
    inputs.usingSecondary = useSecondary;
    inputs.yawVelocityRadPerSec = activeInputs.yawVelocityRadPerSec;
    inputs.odometryYawTimestamps = activeInputs.odometryYawTimestamps;

    if (useSecondary) {
      inputs.yawPosition = activeInputs.yawPosition.plus(secondaryOffset);
      inputs.odometryYawPositions = addOffset(activeInputs.odometryYawPositions, secondaryOffset);
    } else {
      inputs.yawPosition = activeInputs.yawPosition.plus(primaryOffset);
      inputs.odometryYawPositions = addOffset(activeInputs.odometryYawPositions, primaryOffset);
      if (primaryConnected) {
        primaryOffset = decayOffset(primaryOffset, PRIMARY_OFFSET_DECAY_PER_CYCLE_DEG);
      }
    }

    if (inputs.connected) {
      lastPublishedYaw = inputs.yawPosition;
      hasPublishedYaw = true;
    }
  }

  @Override
  public void resetYaw(double yawDegrees) {
    primary.resetYaw(yawDegrees);
    secondary.resetYaw(yawDegrees);
    preferSecondary = false;
    primaryUnhealthyCycles = 0;
    primaryHealthyCycles = 0;
    primaryOffset = new Rotation2d();
    secondaryOffset = new Rotation2d();
    lastPublishedYaw = Rotation2d.fromDegrees(yawDegrees);
    hasPublishedYaw = true;
  }

  private static Rotation2d[] addOffset(Rotation2d[] raw, Rotation2d offset) {
    Rotation2d[] adjusted = new Rotation2d[raw.length];
    for (int i = 0; i < raw.length; i++) {
      adjusted[i] = raw[i].plus(offset);
    }
    return adjusted;
  }

  private static Rotation2d decayOffset(Rotation2d offset, double maxStepDeg) {
    double offsetDeg = offset.getDegrees();
    if (Math.abs(offsetDeg) <= maxStepDeg) {
      return new Rotation2d();
    }
    double nextDeg = offsetDeg - Math.copySign(maxStepDeg, offsetDeg);
    return Rotation2d.fromDegrees(nextDeg);
  }
}
