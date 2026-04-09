package org.Griffins1884.frc2026.simulation.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Deterministic shot replay input scenario used by the local replay verifier. */
public record ShotReplayScenario(
    Pose2d robotPose,
    Translation2d fieldVelocityMetersPerSecond,
    Rotation2d turretYaw,
    double pivotMotorRotations,
    double shooterRpm,
    Translation3d targetPositionMeters,
    Translation3d targetConeTopMeters,
    double openingRadiusMeters,
    double topOpeningRadiusMeters,
    double coneClearanceMeters,
    double stepSeconds,
    int totalSteps,
    int releaseStep) {
  public ShotReplayScenario {
    robotPose = robotPose != null ? robotPose : new Pose2d();
    fieldVelocityMetersPerSecond =
        fieldVelocityMetersPerSecond != null ? fieldVelocityMetersPerSecond : new Translation2d();
    turretYaw = turretYaw != null ? turretYaw : new Rotation2d();
    if (!Double.isFinite(stepSeconds) || stepSeconds <= 0.0) {
      throw new IllegalArgumentException("stepSeconds must be finite and positive");
    }
    if (totalSteps <= 0) {
      throw new IllegalArgumentException("totalSteps must be positive");
    }
    if (releaseStep < 0 || releaseStep >= totalSteps) {
      throw new IllegalArgumentException("releaseStep must fall inside totalSteps");
    }
  }

  public static ShotReplayScenario defaultScenario() {
    return new ShotReplayScenario(
        new Pose2d(2.5, 1.75, Rotation2d.fromDegrees(12.0)),
        new Translation2d(0.65, 0.15),
        Rotation2d.fromDegrees(17.0),
        0.23,
        3600.0,
        new Translation3d(4.2, 1.95, 2.05),
        new Translation3d(4.2, 1.95, 2.45),
        0.35,
        0.45,
        0.2,
        0.02,
        36,
        4);
  }
}
