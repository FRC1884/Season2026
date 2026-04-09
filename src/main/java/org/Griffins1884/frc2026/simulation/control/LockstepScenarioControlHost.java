package org.Griffins1884.frc2026.simulation.control;

import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.replay.ShotReplayScenario;

/** Deterministic actuator generator for a simple shot scenario. */
public final class LockstepScenarioControlHost {
  private final ShotReplayScenario scenario;

  public LockstepScenarioControlHost(ShotReplayScenario scenario) {
    this.scenario = scenario;
  }

  public ActuatorFrame actuatorFrameForStep(int stepId, long simTimeNanos) {
    boolean shooterArmed = stepId >= scenario.releaseStep();
    return new ActuatorFrame(
        simTimeNanos,
        stepId,
        scenario.robotPose(),
        scenario.fieldVelocityMetersPerSecond(),
        scenario.turretYaw().getRadians(),
        scenario.pivotMotorRotations(),
        scenario.shooterRpm(),
        shooterArmed);
  }
}
