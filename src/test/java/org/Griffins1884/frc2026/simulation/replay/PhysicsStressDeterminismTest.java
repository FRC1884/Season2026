package org.Griffins1884.frc2026.simulation.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.junit.jupiter.api.Test;

class PhysicsStressDeterminismTest {
  @Test
  void longDurationArticulatedDriveReplayIsDeterministicAcrossRuns() {
    String first = runLongDriveScenarioHash();
    String second = runLongDriveScenarioHash();
    String third = runLongDriveScenarioHash();
    assertEquals(first, second);
    assertEquals(first, third);
  }

  @Test
  void repeatedWallScrapeAndRapidSteeringRemainFiniteAndDeterministic() {
    DeterministicReplayTrace first = runWallScrapeScenario();
    DeterministicReplayTrace second = runWallScrapeScenario();
    assertEquals(first.sha256Hex(), second.sha256Hex());
    assertTrue(
        first.worldSnapshots().stream()
            .anyMatch(snapshot -> !snapshot.contactPointStates().isEmpty()));
    assertFalse(
        first.worldSnapshots().stream()
            .anyMatch(
                snapshot ->
                    snapshot.rigidBodyStates().stream()
                        .anyMatch(
                            body ->
                                !Double.isFinite(body.positionX())
                                    || !Double.isFinite(body.positionY())
                                    || !Double.isFinite(body.positionZ()))));
  }

  @Test
  void repeatedGamepieceSpawnAndCollisionRemainDeterministic() {
    DeterministicReplayTrace first = runGamepieceImpactScenario();
    DeterministicReplayTrace second = runGamepieceImpactScenario();
    assertEquals(first.sha256Hex(), second.sha256Hex());
    assertTrue(
        first.worldSnapshots().stream()
            .anyMatch(snapshot -> snapshot.activeProjectilePoses().size() >= 3));
  }

  private String runLongDriveScenarioHash() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(2.5, 2.4, Rotation2d.fromDegrees(5.0)));
    List<WorldSnapshot> snapshots = new ArrayList<>();
    for (int step = 0; step < 800; step++) {
      double steerDegrees = 20.0 * Math.sin(step * 0.08);
      double driveRadPerSec = 14.0 + (2.5 * Math.cos(step * 0.05));
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setTurnPosition(
            moduleIndex, Rotation2d.fromDegrees(moduleIndex < 2 ? steerDegrees : -steerDegrees));
        simulation.setDriveVelocity(moduleIndex, driveRadPerSec, 0.0);
      }
      simulation.simulationPeriodic();
      snapshots.add(
          PhysicsSnapshotFactory.fromSimulation(
              simulation, Math.round(simulation.getSimTimeSeconds() * 1_000_000_000.0), step));
    }
    return new DeterministicReplayTrace(List.of(), List.of(), snapshots).sha256Hex();
  }

  private DeterministicReplayTrace runWallScrapeScenario() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(0.35, 0.9, Rotation2d.fromDegrees(90.0)));
    List<WorldSnapshot> snapshots = new ArrayList<>();
    for (int step = 0; step < 260; step++) {
      double steer = step % 2 == 0 ? 90.0 : -90.0;
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setTurnPosition(moduleIndex, Rotation2d.fromDegrees(steer));
        simulation.setDriveVelocity(moduleIndex, 11.0, 0.0);
      }
      simulation.simulationPeriodic();
      snapshots.add(
          PhysicsSnapshotFactory.fromSimulation(
              simulation, Math.round(simulation.getSimTimeSeconds() * 1_000_000_000.0), step));
    }
    return new DeterministicReplayTrace(List.of(), List.of(), snapshots);
  }

  private DeterministicReplayTrace runGamepieceImpactScenario() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(3.1, 2.2, new Rotation2d()));
    List<WorldSnapshot> snapshots = new ArrayList<>();
    for (int step = 0; step < 220; step++) {
      if (step % 40 == 0) {
        simulation.spawnGamePiece(
            new Pose3d(3.6 + (0.1 * step / 40.0), 2.2, 0.22, new Rotation3d()),
            new Vec3(-1.0, 0.0, 0.0));
      }
      double steer = step < 120 ? 0.0 : 180.0;
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setTurnPosition(moduleIndex, Rotation2d.fromDegrees(steer));
        simulation.setDriveVelocity(moduleIndex, step < 120 ? 15.0 : -6.0, 0.0);
      }
      simulation.simulationPeriodic();
      snapshots.add(
          PhysicsSnapshotFactory.fromSimulation(
              simulation, Math.round(simulation.getSimTimeSeconds() * 1_000_000_000.0), step));
    }
    return new DeterministicReplayTrace(List.of(), List.of(), snapshots);
  }
}
