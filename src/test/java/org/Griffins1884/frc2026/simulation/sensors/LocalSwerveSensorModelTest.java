package org.Griffins1884.frc2026.simulation.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class LocalSwerveSensorModelTest {
  @Test
  void sameSeedAndStateProduceIdenticalReleasedSample() {
    SwerveSensorConfig config = SwerveSensorConfig.defaults();
    LocalSwerveSensorModel first = new LocalSwerveSensorModel(config, 1884L);
    LocalSwerveSensorModel second = new LocalSwerveSensorModel(config, 1884L);

    LocalSwerveSensorModel.RawSwerveState raw =
        new LocalSwerveSensorModel.RawSwerveState(
            Rotation2d.fromDegrees(10.0),
            Rotation2d.fromDegrees(2.0),
            Rotation2d.fromDegrees(-1.0),
            0.4,
            0.05,
            -0.03,
            new double[] {0.0, 0.01, 0.02, 0.03, 0.04},
            new Rotation2d[] {
              Rotation2d.fromDegrees(8.0),
              Rotation2d.fromDegrees(8.5),
              Rotation2d.fromDegrees(9.0),
              Rotation2d.fromDegrees(9.5),
              Rotation2d.fromDegrees(10.0)
            },
            filled(1.0),
            filled(2.0),
            filledRotations(Rotation2d.fromDegrees(30.0)),
            filled(0.3),
            filled(30.0 / 360.0));

    first.observe(100_000_000L, raw);
    second.observe(100_000_000L, raw);
    first.releaseReady(200_000_000L);
    second.releaseReady(200_000_000L);

    SwerveSensorSample a = first.current();
    SwerveSensorSample b = second.current();
    assertNotNull(a);
    assertNotNull(b);
    assertEquals(a.yaw().getRadians(), b.yaw().getRadians(), 1e-12);
    assertEquals(a.drivePositionsRad()[0][0], b.drivePositionsRad()[0][0], 1e-12);
    assertEquals(a.turnVelocitiesRadPerSec()[3][4], b.turnVelocitiesRadPerSec()[3][4], 1e-12);
    assertEquals(a.turnPositionsRotations()[2][1], b.turnPositionsRotations()[2][1], 1e-12);
  }

  private static double[][] filled(double value) {
    double[][] data = new double[4][5];
    for (int i = 0; i < data.length; i++) {
      for (int j = 0; j < data[i].length; j++) {
        data[i][j] = value;
      }
    }
    return data;
  }

  private static Rotation2d[][] filledRotations(Rotation2d value) {
    Rotation2d[][] data = new Rotation2d[4][5];
    for (int i = 0; i < data.length; i++) {
      for (int j = 0; j < data[i].length; j++) {
        data[i][j] = value;
      }
    }
    return data;
  }
}
