package org.Griffins1884.frc2026.simulation.sensors;

import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;

/** Converts authoritative world snapshots into latency-aware sensor frames. */
public final class DeterministicSensorEmulator {
  private final long sensorLatencyNanos;
  private final LatencyQueue<SensorFrame> latencyQueue = new LatencyQueue<>();

  public DeterministicSensorEmulator(long sensorLatencyNanos) {
    this.sensorLatencyNanos = Math.max(0L, sensorLatencyNanos);
  }

  public void observe(WorldSnapshot snapshot) {
    SensorFrame frame =
        new SensorFrame(
            snapshot.simTimeNanos(),
            snapshot.stepId(),
            snapshot.predictionAvailable(),
            snapshot.predictionFeasible(),
            false,
            snapshot.activeProjectilePoses().size(),
            snapshot.activeProjectilePoses().size(),
            Double.NaN,
            Double.NaN);
    latencyQueue.enqueue(snapshot.simTimeNanos() + sensorLatencyNanos, frame);
  }

  public List<SensorFrame> releaseReady(long nowNanos) {
    return latencyQueue.releaseReady(nowNanos);
  }
}
