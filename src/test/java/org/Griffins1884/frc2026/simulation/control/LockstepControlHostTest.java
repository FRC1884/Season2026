package org.Griffins1884.frc2026.simulation.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.junit.jupiter.api.Test;

class LockstepControlHostTest {
  @Test
  void lockstepAppliesNewestSensorBeforeSteppingAndCapturesActuators() {
    AtomicInteger pauseCount = new AtomicInteger();
    AtomicInteger resumeCount = new AtomicInteger();
    AtomicInteger stepCount = new AtomicInteger();
    AtomicReference<SensorFrame> applied = new AtomicReference<>();

    LockstepControlHost host =
        new LockstepControlHost(
            0.02,
            8,
            new SimTimeController() {
              @Override
              public void pause() {
                pauseCount.incrementAndGet();
              }

              @Override
              public void resume() {
                resumeCount.incrementAndGet();
              }

              @Override
              public void stepSeconds(double seconds) {
                assertEquals(0.02, seconds, 1e-12);
                stepCount.incrementAndGet();
              }
            },
            applied::set,
            (stepId, simTimeNanos) ->
                new ActuatorFrame(
                    simTimeNanos,
                    stepId,
                    new edu.wpi.first.math.geometry.Pose2d(),
                    new edu.wpi.first.math.geometry.Translation2d(),
                    0.0,
                    0.0,
                    0.0,
                    false));

    host.start();
    assertEquals(1, pauseCount.get());
    assertTrue(
        host.enqueueSensorFrame(
            new SensorFrame(20_000_000L, 1, true, true, false, 0, 0, 0.1, 0.2)));
    assertTrue(
        host.enqueueSensorFrame(
            new SensorFrame(10_000_000L, 0, true, false, false, 0, 0, 0.3, 0.4)));

    ActuatorFrame first = host.runOneTick();
    ActuatorFrame second = host.runOneTick();
    host.stop();

    assertEquals(2, stepCount.get());
    assertEquals(1, resumeCount.get());
    assertEquals(0, first.stepId());
    assertEquals(1, second.stepId());
    assertEquals(1, applied.get().stepId());
  }
}
