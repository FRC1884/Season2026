package org.Griffins1884.frc2026.simulation.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.junit.jupiter.api.Test;

class FieldBoundsAuthorityTest {
  @Test
  void robotAndGamepieceStayInsideAuthoritativeFieldBounds() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(0.3, 0.3, new Rotation2d()));
    simulation.spawnGamePiece(
        new edu.wpi.first.math.geometry.Pose3d(
            0.4, 0.4, 0.2, new edu.wpi.first.math.geometry.Rotation3d()),
        new Vec3(-1.5, -1.2, 0.0));

    for (int step = 0; step < 120; step++) {
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setDriveVelocity(moduleIndex, -10.0, 0.0);
        simulation.setTurnPosition(moduleIndex, Rotation2d.fromDegrees(180.0));
      }
      simulation.simulationPeriodic();
    }

    Pose2d pose = simulation.getPose();
    assertTrue(pose.getX() >= -0.75, "robot x=" + pose.getX());
    assertTrue(pose.getY() >= -0.75, "robot y=" + pose.getY());
    assertTrue(
        pose.getX() <= GlobalConstants.FieldConstants.fieldLength + 0.75, "robot x=" + pose.getX());
    assertTrue(
        pose.getY() <= GlobalConstants.FieldConstants.fieldWidth + 0.75, "robot y=" + pose.getY());
    assertTrue(
        simulation.getGamePiecePoses().stream()
            .allMatch(
                gamepiece ->
                    gamepiece.getX() >= -0.75
                        && gamepiece.getY() >= -0.75
                        && gamepiece.getX() <= GlobalConstants.FieldConstants.fieldLength + 0.75
                        && gamepiece.getY() <= GlobalConstants.FieldConstants.fieldWidth + 0.75));
  }
}
