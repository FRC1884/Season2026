package org.Griffins1884.frc2026.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import org.Griffins1884.frc2026.util.SparkUtil;
import org.griffins1884.sim3d.TerrainAwareSwerveSimulation;
import org.griffins1884.sim3d.TerrainSample;

public class GyroIOSim implements GyroIO {
  private final TerrainAwareSwerveSimulation simulation;

  public GyroIOSim(TerrainAwareSwerveSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    TerrainSample terrainSample = simulation.getTerrainSample();
    inputs.connected = true;
    inputs.yawPosition = simulation.getGyroSimulation().getGyroReading();
    inputs.pitchPosition =
        edu.wpi.first.math.geometry.Rotation2d.fromRadians(terrainSample.pitchRadians());
    inputs.rollPosition =
        edu.wpi.first.math.geometry.Rotation2d.fromRadians(terrainSample.rollRadians());
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(
            simulation.getGyroSimulation().getMeasuredAngularVelocity().in(RadiansPerSecond));
    inputs.pitchVelocityRadPerSec = simulation.getPitchRateRadPerSec();
    inputs.rollVelocityRadPerSec = simulation.getRollRateRadPerSec();

    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = simulation.getGyroSimulation().getCachedGyroReadings();
  }
}
