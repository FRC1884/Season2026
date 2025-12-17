package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import lombok.Getter;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class AprilTagVisionIOLimelight implements VisionIO {
    NetworkTable table;
    // Timeout window for considering the Limelight disconnected, in FPGA microseconds.
    private static final double DISCONNECT_TIMEOUT_MICROS = 250_000.0;

    private final String limelightName;
    private int currentImuMode = -1;
    private final SwerveSubsystem drive;
    @Getter
    private final CameraConstants cameraConstants;
    int imuMode = 1;

    /** Creates a new Limelight vision IO instance. */
    public AprilTagVisionIOLimelight(CameraConstants cameraConstants, SwerveSubsystem drive) {
       this.cameraConstants = cameraConstants;
       this.drive = drive;
       this.limelightName = cameraConstants.cameraName();
        setLLSettings();
    }

    /** Configures Limelight camera poses in robot coordinate system. */
    private void setLLSettings() {
        double[] cameraPose = {
                cameraConstants.robotToCamera().getY(),
                cameraConstants.robotToCamera().getX(),
                cameraConstants.robotToCamera().getZ(),
                0.0,
                cameraConstants.robotToCamera().getRotation().getY(),
                cameraConstants.robotToCamera().getRotation().getZ()
        };

        table.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPose);

    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
//        public boolean connected = false;
//        public TargetObservation latestTargetObservation =
//                new TargetObservation(new Rotation2d(), new Rotation2d());
//        public PoseObservation[] poseObservations = new PoseObservation[0];
//        public int[] tagIds = new int[0];

        inputs.connected = table.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.connected) {
            try {
                LimelightHelpers.PoseEstimate megatag;
                Pose3d robotPose3d;
                if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                    megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                    robotPose3d =
                            LimelightHelpers.toPose3D(
                                    LimelightHelpers.getBotPose_wpiBlue(limelightName));
                } else {
                    megatag = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
                    robotPose3d =
                            LimelightHelpers.toPose3D(
                                    LimelightHelpers.getBotPose_wpiRed(limelightName));
                }
                // Capture latest target offsets when the camera sees targets; otherwise provide zeros.
                if (LimelightHelpers.getTV(limelightName)) {
                    inputs.latestTargetObservation =
                            new TargetObservation(
                                    Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)),
                                    Rotation2d.fromDegrees(LimelightHelpers.getTY(limelightName)));
                } else {
                    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
                }

                MegatagPoseEstimate  megatagPoseEstimate = null;
                FiducialObservation[] fiducialObservation = null;
                // process megatag
                if (megatag != null) {
                    megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    // megatag.tagCount;
                    fiducialObservation = FiducialObservation.fromLimelight(megatag.rawFiducials);
                }

                // Track tag IDs and pose observations for the rest of the robot code to consume.
                Set<Short> tagIds = new HashSet<>();
                List<PoseObservation> poseObservations = new ArrayList<>();

                // Only add a pose observation when we have a valid estimate and pose data.
                if (megatagPoseEstimate != null && megatag.tagCount > 0) {
                    // Average ambiguity across all fiducials helps weight the observation.
                    double ambiguity = calculateAverageAmbiguity(fiducialObservation);

                    // Record which tags contributed to this estimate.
                    for (FiducialObservation fiducial:  fiducialObservation) {
                        tagIds.add((short) fiducial.id());
                    }

                    // Store the pose observation with timestamp, pose, ambiguity, tag count, and avg distance.
                    poseObservations.add(
                            new PoseObservation(
                                    megatag.timestampSeconds,
                                    new Pose3d(megatagPoseEstimate.fieldToRobot().getX(),
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
        double totalAmbiguity = 0.0;
        for (FiducialObservation fiducial : fiducials) {
            totalAmbiguity += fiducial.ambiguity();
        }
        return totalAmbiguity / fiducials.length;
    }

}