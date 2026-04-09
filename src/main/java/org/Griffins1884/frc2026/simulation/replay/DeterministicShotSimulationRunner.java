package org.Griffins1884.frc2026.simulation.replay;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.deterministic.ManualSimulationTimeSource;
import org.Griffins1884.frc2026.simulation.shooter.ProjectileManager;
import org.Griffins1884.frc2026.simulation.shooter.ShotReleaseDetector;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulationConfig;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulator;
import org.Griffins1884.frc2026.simulation.shooter.SimulatedShot;

/** Fixed-step deterministic runner for the local shot simulation stack. */
public final class DeterministicShotSimulationRunner {
  private final ShotSimulationConfig shotSimulationConfig;
  private final ShotSimulator shotSimulator;

  public DeterministicShotSimulationRunner() {
    this(ShotSimulationConfig.defaultConfig());
  }

  public DeterministicShotSimulationRunner(ShotSimulationConfig shotSimulationConfig) {
    this.shotSimulationConfig =
        shotSimulationConfig != null ? shotSimulationConfig : ShotSimulationConfig.defaultConfig();
    shotSimulator = new ShotSimulator(this.shotSimulationConfig);
  }

  public DeterministicReplayTrace run(ShotReplayScenario scenario) {
    ManualSimulationTimeSource timeSource = new ManualSimulationTimeSource();
    ProjectileManager projectileManager =
        new ProjectileManager(shotSimulationConfig.physics(), timeSource);
    ShotReleaseDetector shotReleaseDetector = new ShotReleaseDetector();
    ShotReviewEvents shotReviewEvents = new ShotReviewEvents(timeSource, false);

    List<ActuatorFrame> actuatorFrames = new ArrayList<>(scenario.totalSteps());
    List<SensorFrame> sensorFrames = new ArrayList<>(scenario.totalSteps());
    List<WorldSnapshot> worldSnapshots = new ArrayList<>(scenario.totalSteps());

    for (int step = 0; step < scenario.totalSteps(); step++) {
      SimulatedShot predictedShot =
          shotSimulator
              .solveHubShot(
                  scenario.robotPose(),
                  scenario.fieldVelocityMetersPerSecond(),
                  scenario.turretYaw(),
                  scenario.pivotMotorRotations(),
                  scenario.shooterRpm(),
                  scenario.targetPositionMeters(),
                  scenario.targetConeTopMeters(),
                  scenario.openingRadiusMeters(),
                  scenario.topOpeningRadiusMeters(),
                  scenario.coneClearanceMeters())
              .orElse(null);

      boolean shooterArmed = step >= scenario.releaseStep();
      boolean released =
          shotReleaseDetector.update(
              shooterArmed && predictedShot != null && predictedShot.feasible());
      if (released && predictedShot != null) {
        projectileManager.spawn(predictedShot);
      }

      projectileManager.update(scenario.stepSeconds());
      if (predictedShot != null) {
        shotReviewEvents.recordShotPrediction(
            predictedShot.feasible(),
            predictedShot.closestApproachErrorMeters(),
            predictedShot.timeOfFlightSeconds());
      } else {
        shotReviewEvents.recordShotPrediction(false, Double.NaN, Double.NaN);
      }
      shotReviewEvents.recordShotRelease(released);

      actuatorFrames.add(
          new ActuatorFrame(
              timeSource.nowNanos(),
              step,
              scenario.robotPose(),
              scenario.fieldVelocityMetersPerSecond(),
              scenario.turretYaw().getRadians(),
              scenario.pivotMotorRotations(),
              scenario.shooterRpm(),
              shooterArmed));

      sensorFrames.add(
          new SensorFrame(
              timeSource.nowNanos(),
              step,
              predictedShot != null,
              predictedShot != null && predictedShot.feasible(),
              released,
              projectileManager.activeCount(),
              projectileManager.spawnedCount(),
              predictedShot != null ? predictedShot.closestApproachErrorMeters() : Double.NaN,
              predictedShot != null ? predictedShot.timeOfFlightSeconds() : Double.NaN));

      worldSnapshots.add(
          new WorldSnapshot(
              timeSource.nowNanos(),
              step,
              predictedShot != null,
              predictedShot != null && predictedShot.feasible(),
              predictedShot != null ? predictedShot.releasePose() : null,
              predictedShot != null ? predictedShot.predictedImpactPose() : null,
              Arrays.asList(projectileManager.activeProjectilePoses())));

      timeSource.advanceSeconds(scenario.stepSeconds());
    }

    return new DeterministicReplayTrace(actuatorFrames, sensorFrames, worldSnapshots);
  }
}
