// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.Griffins1884.frc2026.subsystems.swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.Queue;
import org.Griffins1884.frc2026.GlobalConstants;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private double yawOffsetDegrees = 0.0;

  public GyroIONavX() {
    if (GlobalConstants.robotSwerveMotors == GlobalConstants.RobotSwerveMotors.FULLKRACKENS) {
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getAngle);
    } else {
      yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double yawDegrees = -navX.getAngle() + yawOffsetDegrees;
    inputs.connected = navX.isConnected();
    inputs.primaryConnected = inputs.connected;
    inputs.secondaryConnected = false;
    inputs.usingSecondary = false;
    inputs.yawPosition = Rotation2d.fromDegrees(yawDegrees);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value + yawOffsetDegrees))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    if (inputs.odometryYawTimestamps.length == 0) {
      inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
      inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
    }
  }

  @Override
  public void resetYaw(double yawDegrees) {
    navX.zeroYaw();
    yawOffsetDegrees = yawDegrees;
  }
}
