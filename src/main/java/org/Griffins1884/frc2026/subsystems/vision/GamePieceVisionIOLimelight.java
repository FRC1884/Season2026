package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import lombok.Getter;

/** IO implementation for a Limelight camera on a game element detection pipeline. */
public class GamePieceVisionIOLimelight implements VisionIO {
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  @Getter private final CameraConstants cameraConstants;

  /**
   * Creates a new GamePieceVisionIOLimelight.
   *
   * @param cameraConstants The constants associated with this camera.
   */
  public GamePieceVisionIOLimelight(CameraConstants cameraConstants) {
    this.cameraConstants = cameraConstants;

    var table = NetworkTableInstance.getDefault().getTable(cameraConstants.cameraName());
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    table
        .getDoubleArrayTopic("camerapose_robotspace_set")
        .publish()
        .accept(
            new double[] {
              cameraConstants.robotToCamera().getX(),
              cameraConstants.robotToCamera().getY(),
              cameraConstants.robotToCamera().getZ(),
              cameraConstants.robotToCamera().getRotation().getX(),
              cameraConstants.robotToCamera().getRotation().getY(),
              cameraConstants.robotToCamera().getRotation().getZ()
            });
  }

  @Override
  public void updateInputs(VisionIO.VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;
    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight
  }
}
