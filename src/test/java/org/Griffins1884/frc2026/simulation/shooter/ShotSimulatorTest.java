package org.Griffins1884.frc2026.simulation.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ShotSimulatorTest {
  @Test
  void solverProducesFieldSpaceReleaseAndSamples() {
    ShotSimulator simulator = new ShotSimulator(ShotSimulationConfig.defaultConfig());

    SimulatedShot shot =
        simulator
            .solveHubShot(
                new Pose2d(),
                new Translation2d(),
                new Rotation2d(),
                0.2,
                3500.0,
                new Translation3d(4.0, 0.0, 2.05),
                new Translation3d(4.0, 0.0, 2.45),
                0.35,
                0.45,
                0.2)
            .orElseThrow();

    assertTrue(shot.predictedSamplePoses().length > 1);
    assertTrue(shot.releasePose().getZ() > 0.0);
    assertTrue(shot.initialVelocityMetersPerSecond().getNorm() > 0.0);
  }
}
