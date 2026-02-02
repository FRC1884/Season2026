package org.Griffins1884.frc2026.subsystems.vision;

import static org.Griffins1884.frc2026.GlobalConstants.FieldConstants.defaultAprilTagType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param cameraConstants The constants class associated with the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(CameraConstants cameraConstants, Supplier<Pose2d> poseSupplier) {
    super(cameraConstants, () -> 0.0);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(defaultAprilTagType.getLayout());
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibError(0.05, 0.05);
    cameraProperties.setFPS(60);
    cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(80));
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, cameraConstants.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIO.VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
