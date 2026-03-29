package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.Griffins1884.frc2026.util.ballistics.AdvancedBallisticsShotModel;
import org.Griffins1884.frc2026.util.ballistics.ShotModel;
import org.Griffins1884.frc2026.util.ballistics.ShotModelConfig;

/** Converts the shot model into field-space release poses, arcs, and projectile spawns. */
public final class ShotSimulator {
  private static final int MAX_SAMPLE_COUNT = 96;

  private final AdvancedBallisticsShotModel model;
  private final ShotModelConfig config;

  public ShotSimulator(AdvancedBallisticsShotModel model) {
    this.model =
        model != null ? model : new AdvancedBallisticsShotModel(ShotModelConfig.defaultConfig());
    this.config = this.model.config();
  }

  public Optional<SimulatedShot> solveHubShot(
      Pose2d robotPose,
      Translation2d fieldVelocity,
      Translation3d fieldTargetPosition,
      Translation3d fieldConeTopPosition,
      double openingRadiusMeters,
      double topOpeningRadiusMeters,
      double coneClearanceMeters) {
    if (robotPose == null
        || fieldTargetPosition == null
        || fieldConeTopPosition == null
        || !Double.isFinite(robotPose.getX())
        || !Double.isFinite(robotPose.getY())) {
      return Optional.empty();
    }

    Translation2d sanitizedVelocity = fieldVelocity != null ? fieldVelocity : new Translation2d();
    Rotation2d robotHeading = robotPose.getRotation();
    Translation2d robotRelativeTarget =
        fieldTargetPosition
            .toTranslation2d()
            .minus(robotPose.getTranslation())
            .rotateBy(robotHeading.unaryMinus());
    Translation2d robotRelativeConeTop =
        fieldConeTopPosition
            .toTranslation2d()
            .minus(robotPose.getTranslation())
            .rotateBy(robotHeading.unaryMinus());
    Translation2d robotRelativeVelocity = sanitizedVelocity.rotateBy(robotHeading.unaryMinus());
    ShotModel.ShotScenario scenario =
        new ShotModel.ShotScenario(
            new Translation3d(
                robotRelativeTarget.getX(), robotRelativeTarget.getY(), fieldTargetPosition.getZ()),
            robotRelativeVelocity,
            new ShotModel.EntryWindow(
                openingRadiusMeters,
                true,
                fieldConeTopPosition.getZ(),
                topOpeningRadiusMeters,
                true),
            new ShotModel.ClearanceConstraint(
                new Translation3d(
                    robotRelativeTarget.getX(),
                    robotRelativeTarget.getY(),
                    fieldTargetPosition.getZ()),
                0.0,
                new Translation3d(
                    robotRelativeConeTop.getX(),
                    robotRelativeConeTop.getY(),
                    fieldConeTopPosition.getZ()),
                coneClearanceMeters));
    ShotModel.ShotSolution solution = model.solve(scenario);
    ShotModel.ShotPrediction prediction = solution.prediction();

    Rotation2d turretYaw = Rotation2d.fromDegrees(prediction.turretYawDegrees());
    Translation3d robotRelativeExit =
        config.shooterExitPositionMeters(prediction.launchMotorRotations(), turretYaw);
    Translation3d fieldExit = toFieldTranslation(robotPose, robotRelativeExit);

    double exitVelocityMetersPerSecond =
        config.exitVelocityMetersPerSecond(solution.launchCommand().wheelRpm());
    double launchAngleRadians = Math.toRadians(prediction.launchAngleDegrees());
    double horizontalSpeed = exitVelocityMetersPerSecond * Math.cos(launchAngleRadians);
    Translation2d robotRelativeLaunchVelocity =
        new Translation2d(horizontalSpeed, 0.0).rotateBy(turretYaw);
    Translation2d fieldLaunchVelocity =
        robotRelativeLaunchVelocity.rotateBy(robotHeading).plus(sanitizedVelocity);
    Translation3d initialFieldVelocity =
        new Translation3d(
            fieldLaunchVelocity.getX(),
            fieldLaunchVelocity.getY(),
            exitVelocityMetersPerSecond * Math.sin(launchAngleRadians));

    Pose3d releasePose =
        new Pose3d(
            fieldExit,
            new Rotation3d(0.0, -launchAngleRadians, robotHeading.plus(turretYaw).getRadians()));
    Pose3d[] samples = sampleTrajectory(fieldExit, initialFieldVelocity);
    Pose3d impactPose = samples.length > 0 ? samples[samples.length - 1] : releasePose;
    return Optional.of(
        new SimulatedShot(
            prediction.feasible(),
            releasePose,
            initialFieldVelocity,
            samples,
            impactPose,
            solution));
  }

  private Pose3d[] sampleTrajectory(
      Translation3d initialPosition, Translation3d initialVelocityMetersPerSecond) {
    List<Pose3d> samples = new ArrayList<>();
    ProjectileState sample =
        new ProjectileState(initialPosition, initialVelocityMetersPerSecond, 0.0);
    double dtSeconds = config.search().integrationStepSeconds();
    int maxSamples =
        Math.min(
            MAX_SAMPLE_COUNT,
            (int) Math.ceil(config.search().maxSimulationTimeSeconds() / dtSeconds) + 1);
    for (int i = 0; i < maxSamples; i++) {
      samples.add(sample.pose());
      if (!sample.active()) {
        break;
      }
      sample.advance(config.physics(), dtSeconds);
    }
    return samples.toArray(Pose3d[]::new);
  }

  private static Translation3d toFieldTranslation(
      Pose2d robotPose, Translation3d robotRelativeTranslation) {
    Translation2d fieldXY =
        new Translation2d(robotRelativeTranslation.getX(), robotRelativeTranslation.getY())
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
    return new Translation3d(fieldXY.getX(), fieldXY.getY(), robotRelativeTranslation.getZ());
  }
}
