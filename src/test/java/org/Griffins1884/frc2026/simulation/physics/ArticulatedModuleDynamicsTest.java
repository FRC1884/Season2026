package org.Griffins1884.frc2026.simulation.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ArticulatedModuleDynamicsTest {
  @Test
  void moduleBodiesRemainCoupledToChassisWhileDrivingAndSteering() {
    LocalSwervePhysicsSimulation simulation =
        new LocalSwervePhysicsSimulation(new Pose2d(2.5, 2.0, new Rotation2d()));

    for (int step = 0; step < 80; step++) {
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        simulation.setDriveVelocity(moduleIndex, 18.0, 0.0);
        simulation.setTurnPosition(
            moduleIndex, Rotation2d.fromDegrees(moduleIndex < 2 ? 20.0 : -20.0));
      }
      simulation.simulationPeriodic();
    }

    Pose2d chassisPose = simulation.getPose();
    assertTrue(Math.abs(chassisPose.getX() - 2.5) > 0.1, "chassis x=" + chassisPose.getX());
    assertTrue(
        simulation.getBodies().stream()
            .filter(body -> body.name().startsWith("module-"))
            .allMatch(
                body ->
                    Math.hypot(
                            body.position().x() - chassisPose.getX(),
                            body.position().y() - chassisPose.getY())
                        < 1.0),
        "module bodies drifted too far from chassis");
  }
}
