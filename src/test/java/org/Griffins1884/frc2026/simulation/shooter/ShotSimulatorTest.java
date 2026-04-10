package org.Griffins1884.frc2026.simulation.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

  @Test
  void predictedArcIsGravityConsistentWithoutArtificialPathShaping() {
    ShotSimulationConfig config = ShotSimulationConfig.defaultConfig();
    ShotSimulator simulator = new ShotSimulator(config);

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

    Pose3d[] samples = shot.predictedSamplePoses();
    assertTrue(samples.length > 3);
    assertTrue(shot.timeOfFlightSeconds() > 0.0);

    Translation2d horizontalDirection =
        shot.initialVelocityMetersPerSecond()
            .toTranslation2d()
            .div(shot.initialVelocityMetersPerSecond().toTranslation2d().getNorm());
    boolean peaked = false;
    double previousZ = samples[0].getZ();
    double previousForward = 0.0;
    for (int index = 1; index < samples.length; index++) {
      double currentZ = samples[index].getZ();
      if (!peaked && currentZ < previousZ) {
        peaked = true;
      }
      if (peaked) {
        assertTrue(currentZ <= previousZ + 1e-9);
      }
      Translation2d relative =
          samples[index]
              .getTranslation()
              .toTranslation2d()
              .minus(samples[0].getTranslation().toTranslation2d());
      double currentForward =
          (relative.getX() * horizontalDirection.getX())
              + (relative.getY() * horizontalDirection.getY());
      assertTrue(currentForward >= previousForward - 1e-9);
      previousForward = currentForward;
      previousZ = currentZ;
    }

    Pose3d impact = shot.predictedImpactPose();
    assertTrue(Double.isFinite(impact.getZ()));
    assertEquals(samples[samples.length - 1], impact);
  }
}
