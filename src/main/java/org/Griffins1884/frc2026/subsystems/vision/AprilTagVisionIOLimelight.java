package org.Griffins1884.frc2026.subsystems.vision;

import static edu.wpi.first.math.util.Units.radiansToDegrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import lombok.Getter;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class AprilTagVisionIOLimelight implements VisionIO {
  NetworkTable table;
  // Timeout window for considering the Limelight disconnected, in FPGA microseconds.
  private static final double DISCONNECT_TIMEOUT_MICROS = 250_000.0;

  private final String limelightName;
  private final SwerveSubsystem drive;
  @Getter private final CameraConstants cameraConstants;
  int imuMode = 1;

  /** Creates a new Limelight vision IO instance. */
  public AprilTagVisionIOLimelight(CameraConstants cameraConstants, SwerveSubsystem drive) {
    this.cameraConstants = cameraConstants;
    this.drive = drive;
    this.limelightName = cameraConstants.cameraName();
    this.table = NetworkTableInstance.getDefault().getTable(this.limelightName);
    setLLSettings();
  }

  /** Configures Limelight camera poses in robot coordinate system. */
  private void setLLSettings() {
    applyImuMode(1);
    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        cameraConstants.robotToCamera().getX(),
        cameraConstants.robotToCamera().getY(),
        cameraConstants.robotToCamera().getZ(),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getX()),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getY()),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getZ()));
  }

  private void applyImuMode(int desiredMode) {
    if (imuMode == desiredMode) {
      return;
    }
    LimelightHelpers.SetIMUMode(limelightName, desiredMode);
    imuMode = desiredMode;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    //        public boolean connected = false;
    //        public TargetObservation latestTargetObservation =
    //                new TargetObservation(new Rotation2d(), new Rotation2d());
    //        public PoseObservation[] poseObservations = new PoseObservation[0];
    //        public int[] tagIds = new int[0];
    int desiredImuMode = DriverStation.isDisabled() ? 1 : 4;
    applyImuMode(desiredImuMode);

    double yawDeg = drive.getRawGyroRotation().getDegrees();
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red) {
      yawDeg += 180.0;
    }
    yawDeg = MathUtil.inputModulus(yawDeg, -180.0, 180.0);

    LimelightHelpers.SetRobotOrientation(
        limelightName, yawDeg, drive.getYawRateDegreesPerSec(), 0, 0, 0, 0);

    inputs.connected = table.getEntry("tv").getDouble(0) == 1.0;
    inputs.seesTarget = LimelightHelpers.getTV(limelightName);
    inputs.standardDeviations = AprilTagVisionConstants.LIMELIGHT_STANDARD_DEVIATIONS;
    inputs.megatagPoseEstimate = null;
    inputs.pose3d = null;
    inputs.fiducialObservations = new FiducialObservation[0];
    inputs.megatagCount = 0;
    if (inputs.connected) {
      try {
        LimelightHelpers.PoseEstimate megatag;
        Pose3d robotPose3d;
        megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        robotPose3d = LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));
        inputs.pose3d = robotPose3d;
        // Capture latest target offsets when the camera sees targets; otherwise provide zeros.
        if (LimelightHelpers.getTV(limelightName)) {
          inputs.latestTargetObservation =
              new TargetObservation(
                  Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)),
                  Rotation2d.fromDegrees(LimelightHelpers.getTY(limelightName)));
        } else {
          inputs.latestTargetObservation =
              new TargetObservation(new Rotation2d(), new Rotation2d());
        }

        MegatagPoseEstimate megatagPoseEstimate = null;
        FiducialObservation[] fiducialObservation = null;
        // process megatag
        if (megatag != null) {
          megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
          // megatag.tagCount;
          fiducialObservation = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        if (megatagPoseEstimate != null) {
          inputs.megatagPoseEstimate = megatagPoseEstimate;
        }
        if (fiducialObservation != null) {
          inputs.fiducialObservations = fiducialObservation;
        }
        inputs.megatagCount = megatag != null ? megatag.tagCount : 0;
        Logger.recordOutput("AprilTagVision/TagPose", robotPose3d);
        // Track tag IDs and pose observations for the rest of the robot code to consume.
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();

        // Only add a pose observation when we have a valid estimate and pose data.
        if (megatagPoseEstimate != null && megatag.tagCount > 0 && Math.abs(drive.getYawRateDegreesPerSec()) <= cameraConstants.cameraType().noisySpeed) {
          // Average ambiguity across all fiducials helps weight the observation.
          double ambiguity = calculateAverageAmbiguity(fiducialObservation);

          // Record which tags contributed to this estimate.
          for (FiducialObservation fiducial : fiducialObservation) {
            tagIds.add((short) fiducial.id());
          }

          // Store the pose observation with timestamp, pose, ambiguity, tag count, and avg
          // distance.
          poseObservations.add(
              new PoseObservation(
                  megatag.timestampSeconds,
                  new Pose3d(
                      megatagPoseEstimate.fieldToRobot().getX(),
                      megatagPoseEstimate.fieldToRobot().getY(),
                      0.0,
                      new Rotation3d()),
                  ambiguity,
                  megatag.tagCount,
                  megatag.avgTagDist));
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }

  // Compute the mean ambiguity across all detected fiducials; default to 1.0 when none exist.
  private static double calculateAverageAmbiguity(FiducialObservation[] fiducials) {
    if (fiducials == null || fiducials.length == 0) {
      return 1.0;
    }
    double totalAmbiguity = 0.0;
    int count = 0;
    for (FiducialObservation fiducial : fiducials) {
      if (fiducial == null) {
        continue;
      }
      double ambiguity = fiducial.ambiguity();
      if (!Double.isFinite(ambiguity)) {
        continue;
      }
      totalAmbiguity += ambiguity;
      count++;
    }
    return count == 0 ? 1.0 : totalAmbiguity / count;
  }
}
