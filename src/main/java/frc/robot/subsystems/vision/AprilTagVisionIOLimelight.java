package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import lombok.Getter;

/**
 * VisionIO implementation that reads AprilTag pose data from a Limelight, handling both legacy
 * botpose and MegaTag2 pipelines and publishing pose/target data back to the subsystem.
 */
public class AprilTagVisionIOLimelight implements VisionIO {
  // Timeout window for considering the Limelight disconnected, in FPGA microseconds.
  private static final double DISCONNECT_TIMEOUT_MICROS = 250_000.0;

  private final String limelightName;
  // Pose topic when the field is set to blue alliance orientation.
  private final NetworkTableEntry botposeBlueEntry;
  // Pose topic when the field is set to red alliance orientation.
  private final NetworkTableEntry botposeRedEntry;
  // MegaTag2 pose topic (blue orientation).
  private final NetworkTableEntry botposeBlueMegaEntry;
  // MegaTag2 pose topic (red orientation).
  private final NetworkTableEntry botposeRedMegaEntry;
  private int currentImuMode = -1;
  private final SwerveSubsystem drive;
  @Getter private final CameraConstants cameraConstants;

  // Wire up the Limelight topics and push the robot-to-camera transform so the camera can report
  // poses relative to the robot frame.
  public AprilTagVisionIOLimelight(CameraConstants cameraConstants, SwerveSubsystem drive) {
    this.cameraConstants = cameraConstants;
    this.drive = drive;
    limelightName = cameraConstants.cameraName();

    // Pull the base table for this Limelight instance.
    var table = NetworkTableInstance.getDefault().getTable(limelightName);
    botposeBlueEntry = table.getEntry("botpose_wpiblue");
    botposeRedEntry = table.getEntry("botpose_wpired");
    botposeBlueMegaEntry = table.getEntry("botpose_orb_wpiblue");
    botposeRedMegaEntry = table.getEntry("botpose_orb_wpired");

    // Configure the camera with the robot-space transform so it can publish accurate poses.
    table
        .getDoubleArrayTopic("camerapose_robotspace_set")
        .publish()
        .accept(toRobotSpaceArray(cameraConstants.robotToCamera()));
  }

  /** Pull the latest data from the Limelight and populate the VisionIOInputs structure. */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Provide robot yaw (and optional yaw rate) so MegaTag2 can fuse IMU heading.
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        drive.getRotation().getDegrees(),
        drive.getYawRateDegreesPerSec(),
        0,
        0,
        0,
        0);

    // Keep LL4 IMU seeded while disabled (mode 1), then let LL4 use its IMU when enabled (mode 2).
    int desiredImuMode = DriverStation.isDisabled() ? 1 : 2;
    if (desiredImuMode != currentImuMode) {
      LimelightHelpers.SetIMUMode(limelightName, desiredImuMode);
      currentImuMode = desiredImuMode;
    }

    // Determine the current alliance to pick the correct pose topic.
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Prefer MegaTag2 data (supported by Limelight 4), fall back to standard botpose if absent.
    NetworkTableEntry primaryEntry = isRed ? botposeRedMegaEntry : botposeBlueMegaEntry;
    double[] poseArray = primaryEntry.getDoubleArray(new double[0]);
    LimelightHelpers.PoseEstimate estimate =
        isRed
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    // Legacy path: allow disabling MegaTag2 via constants.
    if (!AprilTagVisionConstants.usingMegaTag) {
      primaryEntry = isRed ? botposeRedEntry : botposeBlueEntry;
      poseArray = primaryEntry.getDoubleArray(new double[0]);
      estimate =
          isRed
              ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
              : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    // Treat the camera as connected if it has updated recently.
    inputs.connected =
        (RobotController.getFPGATime() - primaryEntry.getLastChange()) < DISCONNECT_TIMEOUT_MICROS;

    // Capture latest target offsets when the camera sees targets; otherwise provide zeros.
    if (LimelightHelpers.getTV(limelightName)) {
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)),
              Rotation2d.fromDegrees(LimelightHelpers.getTY(limelightName)));
    } else {
      inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    }

    // Track tag IDs and pose observations for the rest of the robot code to consume.
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    // Only add a pose observation when we have a valid estimate and pose data.
    if (estimate != null && estimate.tagCount > 0 && poseArray.length >= 6) {
      // Average ambiguity across all fiducials helps weight the observation.
      double ambiguity = calculateAverageAmbiguity(estimate.rawFiducials);

      // Record which tags contributed to this estimate.
      for (LimelightHelpers.RawFiducial fiducial : safeFiducials(estimate.rawFiducials)) {
        tagIds.add((short) fiducial.id);
      }

      // Store the pose observation with timestamp, pose, ambiguity, tag count, and avg distance.
      poseObservations.add(
          new PoseObservation(
              estimate.timestampSeconds,
              LimelightHelpers.toPose3D(poseArray),
              ambiguity,
              estimate.tagCount,
              estimate.avgTagDist));
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
  }

  // Convert a Transform3d into the NetworkTables array layout expected by Limelight.
  private static double[] toRobotSpaceArray(Transform3d transform) {
    return new double[] {
      transform.getX(),
      transform.getY(),
      transform.getZ(),
      Math.toDegrees(transform.getRotation().getX()),
      Math.toDegrees(transform.getRotation().getY()),
      Math.toDegrees(transform.getRotation().getZ())
    };
  }

  // Defensive helper to avoid null fiducial arrays from Limelight.
  private static LimelightHelpers.RawFiducial[] safeFiducials(
      LimelightHelpers.RawFiducial[] fiducials) {
    return fiducials == null ? new LimelightHelpers.RawFiducial[0] : fiducials;
  }

  // Compute the mean ambiguity across all detected fiducials; default to 1.0 when none exist.
  private static double calculateAverageAmbiguity(LimelightHelpers.RawFiducial[] fiducials) {
    LimelightHelpers.RawFiducial[] safeFids = safeFiducials(fiducials);
    if (safeFids.length == 0) {
      return 1.0;
    }

    double totalAmbiguity = 0.0;
    for (LimelightHelpers.RawFiducial fiducial : safeFids) {
      totalAmbiguity += fiducial.ambiguity;
    }
    return totalAmbiguity / safeFids.length;
  }
}
