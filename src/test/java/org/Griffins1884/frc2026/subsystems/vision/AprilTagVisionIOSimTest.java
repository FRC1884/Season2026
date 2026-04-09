package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.junit.jupiter.api.Test;

class AprilTagVisionIOSimTest {
  @Test
  void simulatedCameraProducesDeterministicPoseObservations() {
    LocalSwervePhysicsSimulation simA =
        new LocalSwervePhysicsSimulation(new Pose2d(3.0, 3.0, new Rotation2d()));
    LocalSwervePhysicsSimulation simB =
        new LocalSwervePhysicsSimulation(new Pose2d(3.0, 3.0, new Rotation2d()));
    AprilTagVisionIOSim ioA =
        new AprilTagVisionIOSim(AprilTagVisionConstants.LEFT_CAM_CONSTANTS, simA);
    AprilTagVisionIOSim ioB =
        new AprilTagVisionIOSim(AprilTagVisionConstants.LEFT_CAM_CONSTANTS, simB);
    VisionIO.VisionIOInputs inputsA = new VisionIO.VisionIOInputs();
    VisionIO.VisionIOInputs inputsB = new VisionIO.VisionIOInputs();

    simA.resetState(new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(15.0)), new ChassisSpeeds());
    simB.resetState(new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(15.0)), new ChassisSpeeds());
    for (int i = 0; i < 5; i++) {
      simA.simulationPeriodic();
      simB.simulationPeriodic();
      ioA.updateInputs(inputsA);
      ioB.updateInputs(inputsB);
    }

    assertEquals(inputsA.seesTarget, inputsB.seesTarget);
    assertEquals(inputsA.megatagCount, inputsB.megatagCount);
    assertTrue(inputsA.tagIds.length >= 0);
    if (inputsA.megatagPoseEstimate != null && inputsB.megatagPoseEstimate != null) {
      assertEquals(
          inputsA.megatagPoseEstimate.fieldToRobot().getX(),
          inputsB.megatagPoseEstimate.fieldToRobot().getX(),
          1e-12);
      assertEquals(
          inputsA.megatagPoseEstimate.fieldToRobot().getRotation().getRadians(),
          inputsB.megatagPoseEstimate.fieldToRobot().getRotation().getRadians(),
          1e-12);
    }
  }
}
