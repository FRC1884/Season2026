package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.commands.AlignConstants;
import org.Griffins1884.frc2026.simulation.replay.ShotReviewEvents;
import org.Griffins1884.frc2026.simulation.shooter.ProjectileManager;
import org.Griffins1884.frc2026.simulation.shooter.ShotReleaseDetector;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulator;
import org.Griffins1884.frc2026.simulation.shooter.SimulatedShot;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperstructureOutcome;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem.IndexerGoal;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.ballistics.AdvancedBallisticsShotModel;
import org.Griffins1884.frc2026.util.ballistics.ShotModelConfig;
import org.littletonrobotics.junction.Logger;

/** Publishes robot components, predicted arcs, and active projectiles for AdvantageScope. */
public final class RobotStateVisualizer {
  private final SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final Superstructure superstructure;
  private final ShotModelConfig shotModelConfig = ShotModelConfig.defaultConfig();
  private final ShotSimulator shotSimulator =
      new ShotSimulator(new AdvancedBallisticsShotModel(shotModelConfig));
  private final ProjectileManager projectileManager =
      new ProjectileManager(shotModelConfig.physics());
  private final ShotReleaseDetector shotReleaseDetector = new ShotReleaseDetector();
  private final GamePiecePosePublisher gamePiecePublisher = new GamePiecePosePublisher();
  private final ShotReviewEvents shotReviewEvents = new ShotReviewEvents();

  public RobotStateVisualizer(
      SwerveSubsystem drive, TurretSubsystem turret, Superstructure superstructure) {
    this.drive = drive;
    this.turret = turret;
    this.superstructure = superstructure;
  }

  public void periodic() {
    Pose2d robotPose = drive != null ? drive.getPose() : null;
    if (!isValidPose(robotPose)) {
      clearPredictionOutputs();
      Logger.recordOutput("FieldSimulation/ActiveProjectiles", new Pose3d[] {});
      return;
    }

    Rotation2d turretYaw =
        turret != null ? Rotation2d.fromRadians(turret.getPositionRad()) : new Rotation2d();
    double shooterPivotRotations =
        superstructure != null && superstructure.getArms().shooterPivot != null
            ? superstructure.getArms().shooterPivot.getPosition()
            : 0.0;

    Pose3d shooterExitPose3d =
        ShooterComponentPublisher.createExitPose(
            robotPose, turretYaw, shooterPivotRotations, shotModelConfig);
    Pose3d turretPose3d =
        TurretComponentPublisher.createPose3d(robotPose, turretYaw, shotModelConfig);
    Pose3d shooterPivotPose3d =
        ShooterComponentPublisher.createPivotPose(
            robotPose, turretYaw, shooterPivotRotations, shotModelConfig);

    Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose3d", new Pose3d(robotPose));
    Logger.recordOutput(
        "FieldSimulation/TurretPose",
        TurretComponentPublisher.createPose2d(robotPose, turretYaw, shotModelConfig));
    Logger.recordOutput("FieldSimulation/TurretComponentPose3d", new Pose3d[] {turretPose3d});
    Logger.recordOutput(
        "FieldSimulation/ShooterPivotComponentPose3d", new Pose3d[] {shooterPivotPose3d});
    Logger.recordOutput("FieldSimulation/ShooterExitPose3d", new Pose3d[] {shooterExitPose3d});
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPoses",
        new Pose3d[] {turretPose3d, shooterPivotPose3d, shooterExitPose3d});

    SimulatedShot predictedShot = null;
    if (superstructure != null && superstructure.isInAllianceZone()) {
      predictedShot =
          shotSimulator
              .solveHubShot(
                  robotPose,
                  sanitize(drive != null ? drive.getFieldVelocity() : null),
                  getHubTarget(robotPose),
                  getHubConeTop(robotPose),
                  GlobalConstants.FieldConstants.Hub.innerOpeningRadius,
                  GlobalConstants.FieldConstants.Hub.topOpeningRadius,
                  GlobalConstants.FieldConstants.Hub.coneClearanceMargin)
              .orElse(null);
    }

    if (predictedShot != null) {
      Pose3d targetPose = new Pose3d(getHubTarget(robotPose), shooterExitPose3d.getRotation());
      gamePiecePublisher.publishPredictedArc(predictedShot.predictedSamplePoses());
      gamePiecePublisher.publishReleasePose(predictedShot.releasePose());
      gamePiecePublisher.publishImpactPose(predictedShot.predictedImpactPose());
      Logger.recordOutput("FieldSimulation/TargetPose3d", new Pose3d[] {targetPose});
      gamePiecePublisher.publishShotMarkers(
          predictedShot.releasePose(), predictedShot.predictedImpactPose(), targetPose);
      shotReviewEvents.recordShotPrediction(
          predictedShot.feasible(),
          predictedShot.solution().prediction().closestApproachErrorMeters(),
          predictedShot.solution().prediction().timeOfFlightSeconds());
    } else {
      clearPredictionOutputs();
      shotReviewEvents.recordShotPrediction(false, Double.NaN, Double.NaN);
    }

    boolean released = false;
    if (superstructure != null) {
      SuperstructureOutcome outcome = superstructure.getLatestOutcome();
      boolean shooterReady =
          superstructure.getRollers().shooter == null
              || superstructure.getRollers().shooter.isAtGoal();
      boolean armed =
          predictedShot != null
              && predictedShot.feasible()
              && superstructure.hasBall()
              && shooterReady
              && outcome.indexerGoal() == IndexerGoal.FORWARD
              && outcome.shooterTargetVelocityRpm() > 1.0
              && (outcome.state() == SuperState.SHOOTING
                  || outcome.state() == SuperState.SHOOT_INTAKE
                  || outcome.state() == SuperState.FERRYING);
      released = shotReleaseDetector.update(armed);
      if (released) {
        projectileManager.spawn(predictedShot);
      }
    }

    projectileManager.update(AlignConstants.LOOP_PERIOD_SEC);
    gamePiecePublisher.publishActiveProjectiles(projectileManager.activeProjectilePoses());
    shotReviewEvents.recordShotRelease(released);
    Logger.recordOutput("FieldSimulation/ProjectileCount", projectileManager.activeCount());
    Logger.recordOutput("FieldSimulation/ProjectileSpawnCount", projectileManager.spawnedCount());
  }

  public void reset() {
    projectileManager.clear();
    shotReleaseDetector.reset();
    clearPredictionOutputs();
    gamePiecePublisher.publishActiveProjectiles(new Pose3d[] {});
  }

  private void clearPredictionOutputs() {
    gamePiecePublisher.publishPredictedArc(new Pose3d[] {});
    gamePiecePublisher.publishReleasePose(null);
    gamePiecePublisher.publishImpactPose(null);
    gamePiecePublisher.publishShotMarkers(null, null, null);
    Logger.recordOutput("FieldSimulation/TargetPose3d", new Pose3d[] {});
  }

  private static boolean isValidPose(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  private static Translation2d sanitize(Translation2d velocity) {
    if (velocity == null
        || !Double.isFinite(velocity.getX())
        || !Double.isFinite(velocity.getY())) {
      return new Translation2d();
    }
    return velocity;
  }

  private static Translation3d getHubTarget(Pose2d pose) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElseGet(() -> inferAllianceFromPose(pose));
    return alliance == DriverStation.Alliance.Blue
        ? GlobalConstants.FieldConstants.Hub.innerCenterPoint
        : new Translation3d(
            GlobalConstants.FieldConstants.fieldLength
                - GlobalConstants.FieldConstants.Hub.innerCenterPoint.getX(),
            GlobalConstants.FieldConstants.Hub.innerCenterPoint.getY(),
            GlobalConstants.FieldConstants.Hub.innerCenterPoint.getZ());
  }

  private static Translation3d getHubConeTop(Pose2d pose) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElseGet(() -> inferAllianceFromPose(pose));
    return alliance == DriverStation.Alliance.Blue
        ? GlobalConstants.FieldConstants.Hub.topCenterPoint
        : new Translation3d(
            GlobalConstants.FieldConstants.fieldLength
                - GlobalConstants.FieldConstants.Hub.topCenterPoint.getX(),
            GlobalConstants.FieldConstants.Hub.topCenterPoint.getY(),
            GlobalConstants.FieldConstants.Hub.topCenterPoint.getZ());
  }

  private static DriverStation.Alliance inferAllianceFromPose(Pose2d pose) {
    return pose != null && pose.getX() <= GlobalConstants.FieldConstants.fieldLength * 0.5
        ? DriverStation.Alliance.Blue
        : DriverStation.Alliance.Red;
  }
}
