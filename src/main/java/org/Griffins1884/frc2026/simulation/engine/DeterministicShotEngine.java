package org.Griffins1884.frc2026.simulation.engine;

import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.deterministic.ManualSimulationTimeSource;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.Griffins1884.frc2026.simulation.replay.PhysicsSnapshotFactory;
import org.Griffins1884.frc2026.simulation.shooter.ShotReleaseDetector;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulationConfig;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulator;
import org.Griffins1884.frc2026.simulation.shooter.SimulatedShot;

/** Authoritative fixed-step shot engine with deterministic sensor emission. */
public final class DeterministicShotEngine {
  private final ShotSimulationConfig shotSimulationConfig;
  private final ShotEngineConfig engineConfig;
  private final ManualSimulationTimeSource timeSource = new ManualSimulationTimeSource();
  private final ShotSimulator shotSimulator;
  private final ShotReleaseDetector shotReleaseDetector = new ShotReleaseDetector();
  private LocalSwervePhysicsSimulation simulation;
  private boolean lastArmed = false;
  private long lastSimTimeNanos = -1L;

  public DeterministicShotEngine() {
    this(ShotSimulationConfig.defaultConfig(), ShotEngineConfig.defaultConfig(), 0L);
  }

  public DeterministicShotEngine(
      ShotSimulationConfig shotSimulationConfig,
      ShotEngineConfig engineConfig,
      long sensorLatencyNanos) {
    this.shotSimulationConfig =
        shotSimulationConfig != null ? shotSimulationConfig : ShotSimulationConfig.defaultConfig();
    this.engineConfig = engineConfig != null ? engineConfig : ShotEngineConfig.defaultConfig();
    shotSimulator = new ShotSimulator(this.shotSimulationConfig);
  }

  public synchronized EngineStepResult step(ActuatorFrame frame) {
    if (simulation == null) {
      simulation = new LocalSwervePhysicsSimulation(frame.robotPose());
    }
    timeSource.setNanos(frame.simTimeNanos());
    double dtSeconds = 0.0;
    if (lastSimTimeNanos >= 0L && frame.simTimeNanos() >= lastSimTimeNanos) {
      dtSeconds = (frame.simTimeNanos() - lastSimTimeNanos) / 1_000_000_000.0;
    }
    lastSimTimeNanos = frame.simTimeNanos();

    SimulatedShot predictedShot =
        shotSimulator
            .solveHubShot(
                frame.robotPose(),
                frame.fieldVelocityMetersPerSecond(),
                edu.wpi.first.math.geometry.Rotation2d.fromRadians(frame.turretYawRadians()),
                frame.pivotMotorRotations(),
                frame.shooterRpm(),
                engineConfig.targetPositionMeters(),
                engineConfig.targetConeTopMeters(),
                engineConfig.openingRadiusMeters(),
                engineConfig.topOpeningRadiusMeters(),
                engineConfig.coneClearanceMeters())
            .orElse(null);

    boolean released =
        shotReleaseDetector.update(
            frame.shooterArmed() && predictedShot != null && predictedShot.feasible());
    double driveCommandRadPerSec =
        frame.fieldVelocityMetersPerSecond().getNorm()
            / org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.getWheelRadiusMeters();
    double driveHeading =
        Math.atan2(
            frame.fieldVelocityMetersPerSecond().getY(),
            frame.fieldVelocityMetersPerSecond().getX());
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      simulation.setTurnPosition(
          moduleIndex, edu.wpi.first.math.geometry.Rotation2d.fromRadians(driveHeading));
      simulation.setDriveVelocity(moduleIndex, driveCommandRadPerSec, 0.0);
    }
    if (released && predictedShot != null && !lastArmed) {
      simulation.spawnGamePiece(
          predictedShot.releasePose(),
          new Vec3(
              predictedShot.initialVelocityMetersPerSecond().getX(),
              predictedShot.initialVelocityMetersPerSecond().getY(),
              predictedShot.initialVelocityMetersPerSecond().getZ()));
    }
    lastArmed = frame.shooterArmed();
    simulation.simulationPeriodic();

    WorldSnapshot snapshot =
        PhysicsSnapshotFactory.fromSimulation(simulation, frame.simTimeNanos(), frame.stepId());
    WorldSnapshot enrichedSnapshot =
        new WorldSnapshot(
            snapshot.simTimeNanos(),
            snapshot.stepId(),
            predictedShot != null,
            predictedShot != null && predictedShot.feasible(),
            predictedShot != null ? predictedShot.releasePose() : null,
            predictedShot != null ? predictedShot.predictedImpactPose() : null,
            snapshot.activeProjectilePoses(),
            snapshot.rigidBodyStates(),
            snapshot.contactPointStates(),
            snapshot.wheelContactStates());
    List<SensorFrame> sensorFrames =
        java.util.List.of(
            new SensorFrame(
                frame.simTimeNanos(),
                frame.stepId(),
                predictedShot != null,
                predictedShot != null && predictedShot.feasible(),
                released,
                simulation.getGamePiecePoses().size(),
                simulation.getGamePiecePoses().size(),
                predictedShot != null ? predictedShot.closestApproachErrorMeters() : Double.NaN,
                predictedShot != null ? predictedShot.timeOfFlightSeconds() : Double.NaN));
    return new EngineStepResult(sensorFrames, enrichedSnapshot);
  }
}
