package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;

/** Publishes projectile and shot-arc poses for AdvantageScope. */
public final class GamePiecePosePublisher {
  public void publishPredictedArc(Pose3d[] predictedArc) {
    Logger.recordOutput(
        "FieldSimulation/PredictedShotArc", predictedArc != null ? predictedArc : new Pose3d[] {});
  }

  public void publishActiveProjectiles(Pose3d[] projectilePoses) {
    Logger.recordOutput(
        "FieldSimulation/ActiveProjectiles",
        projectilePoses != null ? projectilePoses : new Pose3d[] {});
  }

  public void publishImpactPose(Pose3d impactPose) {
    Logger.recordOutput(
        "FieldSimulation/PredictedImpactPose",
        impactPose != null ? new Pose3d[] {impactPose} : new Pose3d[] {});
  }

  public void publishReleasePose(Pose3d releasePose) {
    Logger.recordOutput(
        "FieldSimulation/ShotReleasePose",
        releasePose != null ? new Pose3d[] {releasePose} : new Pose3d[] {});
  }

  public void publishShotMarkers(Pose3d releasePose, Pose3d impactPose, Pose3d targetPose) {
    Pose3d[] markers;
    if (releasePose != null && impactPose != null && targetPose != null) {
      markers = new Pose3d[] {releasePose, impactPose, targetPose};
    } else if (releasePose != null && impactPose != null) {
      markers = new Pose3d[] {releasePose, impactPose};
    } else if (releasePose != null && targetPose != null) {
      markers = new Pose3d[] {releasePose, targetPose};
    } else if (impactPose != null && targetPose != null) {
      markers = new Pose3d[] {impactPose, targetPose};
    } else if (releasePose != null) {
      markers = new Pose3d[] {releasePose};
    } else if (impactPose != null) {
      markers = new Pose3d[] {impactPose};
    } else if (targetPose != null) {
      markers = new Pose3d[] {targetPose};
    } else {
      markers = new Pose3d[] {};
    }
    Logger.recordOutput("FieldSimulation/ShotMarkers", markers);
  }
}
