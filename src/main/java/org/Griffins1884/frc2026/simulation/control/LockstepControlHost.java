package org.Griffins1884.frc2026.simulation.control;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.runtime.DeterministicRingBuffer;

/** Generic lockstep host that applies sensor frames, steps time, and captures actuators. */
public final class LockstepControlHost {
  private final double stepSeconds;
  private final SimTimeController timeController;
  private final SensorFrameApplicator sensorApplicator;
  private final ActuatorFrameSupplier actuatorFrameSupplier;
  private final DeterministicRingBuffer<SensorFrame> inboundSensorFrames;
  private int nextStepId = 0;

  public LockstepControlHost(
      double stepSeconds,
      int queueCapacity,
      SimTimeController timeController,
      SensorFrameApplicator sensorApplicator,
      ActuatorFrameSupplier actuatorFrameSupplier) {
    this.stepSeconds = stepSeconds;
    this.timeController = timeController;
    this.sensorApplicator = sensorApplicator;
    this.actuatorFrameSupplier = actuatorFrameSupplier;
    inboundSensorFrames = new DeterministicRingBuffer<>(queueCapacity);
  }

  public void start() {
    timeController.pause();
  }

  public void stop() {
    timeController.resume();
  }

  public boolean enqueueSensorFrame(SensorFrame frame) {
    if (frame == null) {
      return false;
    }
    return inboundSensorFrames.offer(frame);
  }

  public ActuatorFrame runOneTick() {
    int stepId = nextStepId++;
    selectSensorFrameFor(stepId).ifPresent(sensorApplicator::apply);
    long simTimeNanos = Math.round(stepId * stepSeconds * 1_000_000_000.0);
    timeController.stepSeconds(stepSeconds);
    return actuatorFrameSupplier.capture(stepId, simTimeNanos);
  }

  private Optional<SensorFrame> selectSensorFrameFor(int controlStepId) {
    SensorFrame newestApplicable = null;
    List<SensorFrame> futureFrames = new ArrayList<>();
    Optional<SensorFrame> next;
    while ((next = inboundSensorFrames.poll()).isPresent()) {
      SensorFrame frame = next.orElseThrow();
      if (frame.stepId() <= controlStepId) {
        newestApplicable = frame;
      } else {
        futureFrames.add(frame);
      }
    }
    for (SensorFrame frame : futureFrames) {
      boolean offered = inboundSensorFrames.offer(frame);
      if (!offered) {
        throw new IllegalStateException(
            "Sensor frame queue overflow while restoring future frames");
      }
    }
    return Optional.ofNullable(newestApplicable);
  }
}
