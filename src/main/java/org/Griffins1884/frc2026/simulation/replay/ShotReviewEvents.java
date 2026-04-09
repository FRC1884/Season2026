package org.Griffins1884.frc2026.simulation.replay;

import org.Griffins1884.frc2026.simulation.deterministic.SimulationTimeSource;
import org.Griffins1884.frc2026.simulation.deterministic.WallClockSimulationTimeSource;
import org.littletonrobotics.junction.Logger;

/** Records shot-review markers that can be aligned with replay and external video. */
public final class ShotReviewEvents {
  private final SimulationTimeSource timeSource;
  private final boolean publishToLogger;
  private int releaseCount = 0;
  private double lastReleaseTimestampSec = Double.NaN;

  public ShotReviewEvents() {
    this(new WallClockSimulationTimeSource(), true);
  }

  public ShotReviewEvents(SimulationTimeSource timeSource, boolean publishToLogger) {
    this.timeSource = timeSource != null ? timeSource : new WallClockSimulationTimeSource();
    this.publishToLogger = publishToLogger;
  }

  public void recordShotRelease(boolean released) {
    if (released) {
      releaseCount++;
      lastReleaseTimestampSec = timeSource.nowSeconds();
    }
    if (publishToLogger) {
      Logger.recordOutput("ShotReview/Released", released);
      Logger.recordOutput("ShotReview/ReleaseCount", releaseCount);
      Logger.recordOutput("ShotReview/LastReleaseTimestampSec", lastReleaseTimestampSec);
    }
  }

  public void recordShotPrediction(boolean feasible, double errorMeters, double flightTimeSec) {
    if (publishToLogger) {
      Logger.recordOutput("ShotReview/PredictionFeasible", feasible);
      Logger.recordOutput("ShotReview/PredictionErrorMeters", errorMeters);
      Logger.recordOutput("ShotReview/PredictionFlightTimeSec", flightTimeSec);
    }
  }
}
