package org.Griffins1884.frc2026.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;

public class GyroIOSim implements GyroIO {
  private final LocalSwervePhysicsSimulation simulation;

  public GyroIOSim(LocalSwervePhysicsSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = simulation.getYawRotation();
    inputs.pitchPosition = simulation.getPitchRotation();
    inputs.rollPosition = simulation.getRollRotation();
    inputs.yawVelocityRadPerSec = simulation.getYawRateRadPerSec();
    inputs.pitchVelocityRadPerSec = simulation.getPitchRateRadPerSec();
    inputs.rollVelocityRadPerSec = simulation.getRollRateRadPerSec();
    inputs.odometryYawTimestamps = simulation.getCachedTimestamps();
    inputs.odometryYawPositions = simulation.getCachedYawPositions();
  }

  @Override
  public void resetYaw(double yawDegrees) {
    simulation.resetYaw(Rotation2d.fromDegrees(yawDegrees));
  }
}
