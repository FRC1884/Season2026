package org.Griffins1884.frc2026.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.Griffins1884.frc2026.simulation.drive.Season2026DriveSimulation;
import org.Griffins1884.frc2026.util.SparkUtil;

public class GyroIOSim implements GyroIO {
  private final Season2026DriveSimulation simulation;

  public GyroIOSim(Season2026DriveSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    Season2026DriveSimulation.TerrainSampleSnapshot terrainSample = simulation.terrainSample();
    inputs.connected = true;
    inputs.yawPosition = simulation.gyroReading();
    inputs.pitchPosition =
        edu.wpi.first.math.geometry.Rotation2d.fromRadians(terrainSample.pitchRadians());
    inputs.rollPosition =
        edu.wpi.first.math.geometry.Rotation2d.fromRadians(terrainSample.rollRadians());
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(simulation.measuredAngularVelocity().in(RadiansPerSecond));
    inputs.pitchVelocityRadPerSec = simulation.pitchRateRadPerSec();
    inputs.rollVelocityRadPerSec = simulation.rollRateRadPerSec();

    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = simulation.cachedGyroReadings();
  }

  @Override
  public void resetYaw(double yawDegrees) {
    simulation.setGyroRotation(Rotation2d.fromDegrees(yawDegrees));
  }
}
