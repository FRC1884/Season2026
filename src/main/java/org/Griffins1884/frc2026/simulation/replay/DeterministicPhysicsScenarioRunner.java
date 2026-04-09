package org.Griffins1884.frc2026.simulation.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;

/** Runs a deterministic contact-rich local physics scenario for replay validation. */
public final class DeterministicPhysicsScenarioRunner {
  public DeterministicReplayTrace runDefaultScenario() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(2.4, 2.0, new Rotation2d()));
    simulation.spawnGamePiece(
        new Pose3d(3.7, 2.0, 0.15, new edu.wpi.first.math.geometry.Rotation3d()), Vec3.ZERO);

    List<ActuatorFrame> actuatorFrames = new ArrayList<>();
    List<SensorFrame> sensorFrames = new ArrayList<>();
    List<WorldSnapshot> worldSnapshots = new ArrayList<>();

    double wheelCommand =
        (SwerveConstants.MAX_LINEAR_SPEED / SwerveConstants.getWheelRadiusMeters()) * 0.8;
    for (int step = 0; step < 90; step++) {
      double command = step < 55 ? wheelCommand : 0.0;
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setTurnPosition(moduleIndex, new Rotation2d());
        simulation.setDriveVelocity(moduleIndex, command, 0.0);
      }
      simulation.simulationPeriodic();
      long simTimeNanos =
          Math.round(
              (step + 1) * LocalSwervePhysicsSimulation.LOOP_PERIOD_SECONDS * 1_000_000_000.0);
      actuatorFrames.add(
          new ActuatorFrame(
              simTimeNanos,
              step,
              simulation.getPose(),
              new edu.wpi.first.math.geometry.Translation2d(),
              0.0,
              0.0,
              0.0,
              command != 0.0));
      sensorFrames.add(
          new SensorFrame(
              simTimeNanos,
              step,
              false,
              false,
              false,
              simulation.getGamePiecePoses().size(),
              simulation.getGamePiecePoses().size(),
              Double.NaN,
              Double.NaN));
      worldSnapshots.add(PhysicsSnapshotFactory.fromSimulation(simulation, simTimeNanos, step));
    }

    return new DeterministicReplayTrace(actuatorFrames, sensorFrames, worldSnapshots);
  }
}
