package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.junit.jupiter.api.Test;

class AprilTagVisionIOLimelightTest {
  @Test
  void connectedUsesHeartbeatNotTargetVisibility() {
    String cameraName = "limelight-test";
    VisionIO.CameraConstants constants =
        new VisionIO.CameraConstants(cameraName, new Transform3d(), VisionIO.CameraType.LIMELIGHT);

    SwerveSubsystem drive = mock(SwerveSubsystem.class);
    when(drive.getRawGyroRotation()).thenReturn(new Rotation2d());
    when(drive.getYawRateDegreesPerSec()).thenReturn(0.0);

    AprilTagVisionIOLimelight io = new AprilTagVisionIOLimelight(constants, drive);

    NetworkTable table = NetworkTableInstance.getDefault().getTable(cameraName);
    var latencyPublisher = table.getDoubleTopic("tl").publish();
    latencyPublisher.set(0.0);
    table.getEntry("tv").setDouble(0.0);
    NetworkTableInstance.getDefault().flush();

    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    io.updateInputs(inputs);

    assertTrue(inputs.connected);
    assertFalse(inputs.seesTarget);

    verify(drive, atLeastOnce()).getRawGyroRotation();
    verify(drive, never()).getPose();

    latencyPublisher.close();
  }
}
