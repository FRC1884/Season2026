package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.junit.jupiter.api.Test;

class AprilTagVisionIOSimOcclusionTest {
  @Test
  void occlusionTransitionsAreDeterministicAcrossRuns() {
    var first = runOcclusionSequence();
    var second = runOcclusionSequence();
    assertEquals(first, second);
  }

  private java.util.List<Boolean> runOcclusionSequence() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(3.0, 3.0, new Rotation2d()));
    AprilTagVisionIOSim io =
        new AprilTagVisionIOSim(AprilTagVisionConstants.LEFT_CAM_CONSTANTS, simulation);
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    java.util.List<Boolean> visible = new java.util.ArrayList<>();

    var blocker =
        simulation.getBodies().stream()
            .filter(body -> "left-bump-blue".equals(body.name()))
            .findFirst()
            .orElseThrow();

    for (int step = 0; step < 20; step++) {
      if (step < 10) {
        blocker.setPose(new Vec3(2.9, 3.0, 0.35), Quat.IDENTITY);
      } else {
        blocker.setPose(new Vec3(5.0, 5.0, 0.35), Quat.IDENTITY);
      }
      simulation.simulationPeriodic();
      io.updateInputs(inputs);
      visible.add(inputs.seesTarget);
    }
    return visible;
  }
}
