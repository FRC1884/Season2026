package org.Griffins1884.frc2026.simulation.replay;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Records shot-review markers that can be aligned with replay and external video. */
public final class ShotReviewEvents {
  private int releaseCount = 0;
  private double lastReleaseTimestampSec = Double.NaN;

  public void recordShotRelease(boolean released) {
    if (released) {
      releaseCount++;
      lastReleaseTimestampSec = Timer.getFPGATimestamp();
    }
    Logger.recordOutput("ShotReview/Released", released);
    Logger.recordOutput("ShotReview/ReleaseCount", releaseCount);
    Logger.recordOutput("ShotReview/LastReleaseTimestampSec", lastReleaseTimestampSec);
  }

  public void recordShotPrediction(boolean feasible, double errorMeters, double flightTimeSec) {
    Logger.recordOutput("ShotReview/PredictionFeasible", feasible);
    Logger.recordOutput("ShotReview/PredictionErrorMeters", errorMeters);
    Logger.recordOutput("ShotReview/PredictionFlightTimeSec", flightTimeSec);
  }
}
