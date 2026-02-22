package org.Griffins1884.frc2026.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SwerveSubsystemResetAndRecoveryTest {
  private static final double EPSILON = 1e-6;

  private FakeGyro gyro;
  private SwerveSubsystem subsystem;

  @BeforeAll
  void setUp() {
    gyro = new FakeGyro();
    subsystem = newSubsystem(gyro);
  }

  @Test
  void odometryResetListenerRunsForDirectAndAllianceZeroResets() {
    gyro.resetCount = 0;
    gyro.lastResetYawDegrees = 0.0;

    AtomicInteger resetNotifications = new AtomicInteger();
    subsystem.setOdometryResetListener(resetNotifications::incrementAndGet);

    subsystem.resetOdometry(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0)));
    subsystem.zeroGyroAndOdometryToAllianceWall(Alliance.Blue);

    assertEquals(2, resetNotifications.get());
    assertEquals(1, gyro.resetCount);
    assertEquals(180.0, gyro.lastResetYawDegrees, EPSILON);
  }

  @Test
  void visionRecoveryAlignsRawHeadingWhenEstimatorPoseIsInvalid() {
    subsystem.setOdometryResetListener(() -> {});

    subsystem.resetOdometry(new Pose2d(Double.NaN, 0.0, Rotation2d.fromDegrees(5.0)));
    assertTrue(Double.isNaN(subsystem.getPose().getX()));

    Pose2d visionPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(37.0));
    subsystem.accept(visionPose, 1.0, VecBuilder.fill(0.2, 0.2, 0.2));

    assertEquals(37.0, subsystem.getRawGyroRotation().getDegrees(), EPSILON);
    assertEquals(3.0, subsystem.getPose().getX(), EPSILON);
    assertEquals(4.0, subsystem.getPose().getY(), EPSILON);
    assertEquals(37.0, subsystem.getPose().getRotation().getDegrees(), EPSILON);
  }

  @Test
  void fieldMotionSampleValidityRequiresSettledTiming() throws InterruptedException {
    subsystem.periodic();
    assertFalse(subsystem.isFieldMotionSampleValid());

    for (int i = 0; i < 10 && !subsystem.isFieldMotionSampleValid(); i++) {
      Thread.sleep(2);
      subsystem.periodic();
    }

    assertTrue(subsystem.isFieldMotionSampleValid());
    assertTrue(subsystem.getFieldMotionSampleAgeSec() >= 0.0);
  }

  private static SwerveSubsystem newSubsystem(FakeGyro gyro) {
    ModuleIO module = new ModuleIO() {};
    return new SwerveSubsystem(gyro, module, module, module, module);
  }

  private static class FakeGyro implements GyroIO {
    private int resetCount = 0;
    private double lastResetYawDegrees = 0.0;

    @Override
    public void resetYaw(double yawDegrees) {
      resetCount++;
      lastResetYawDegrees = yawDegrees;
    }
  }
}
