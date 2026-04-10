package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.GlobalConstants.RobotMode;
import org.Griffins1884.frc2026.commands.AlignConstants;
import org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel;
import org.Griffins1884.frc2026.simulation.replay.ShotReviewEvents;
import org.Griffins1884.frc2026.simulation.shooter.ProjectileManager;
import org.Griffins1884.frc2026.simulation.shooter.ProjectileManager.ProjectileSnapshot;
import org.Griffins1884.frc2026.simulation.shooter.ShotReleaseDetector;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulationConfig;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulator;
import org.Griffins1884.frc2026.simulation.shooter.SimulatedShot;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperstructureOutcome;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem.IndexerGoal;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;

/** Publishes robot components, predicted arcs, and active projectiles for AdvantageScope. */
public final class RobotStateVisualizer {
  private final SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final Superstructure superstructure;
  private final Supplier<Pose3d> authoritativeRobotPose3dSupplier;
  private final ShotSimulationConfig shotSimulationConfig = ShotSimulationConfig.defaultConfig();
  private final ShotSimulator shotSimulator = new ShotSimulator(shotSimulationConfig);
  private final ProjectileManager projectileManager =
      new ProjectileManager(shotSimulationConfig.physics());
  private final ShotReleaseDetector shotReleaseDetector = new ShotReleaseDetector();
  private final ShotTelemetryAccumulator shotTelemetryAccumulator = new ShotTelemetryAccumulator();
  private final GamePiecePosePublisher gamePiecePublisher = new GamePiecePosePublisher();
  private final ShotReviewEvents shotReviewEvents = new ShotReviewEvents();
  private RenderTelemetry lastRenderTelemetry = new RenderTelemetry(null, null, null, false);

  public RobotStateVisualizer(
      SwerveSubsystem drive, TurretSubsystem turret, Superstructure superstructure) {
    this(drive, turret, superstructure, null);
  }

  public RobotStateVisualizer(
      SwerveSubsystem drive,
      TurretSubsystem turret,
      Superstructure superstructure,
      Supplier<Pose3d> authoritativeRobotPose3dSupplier) {
    this.drive = drive;
    this.turret = turret;
    this.superstructure = superstructure;
    this.authoritativeRobotPose3dSupplier = authoritativeRobotPose3dSupplier;
  }

  public void periodic() {
    Pose2d robotPose = drive != null ? drive.getPose() : null;
    if (!isValidPose(robotPose)) {
      clearPredictionOutputs();
      Logger.recordOutput("FieldSimulation/ActiveProjectiles", new Pose3d[] {});
      lastRenderTelemetry = new RenderTelemetry(null, null, null, false);
      return;
    }

    Rotation2d turretYaw =
        turret != null ? Rotation2d.fromRadians(turret.getPositionRad()) : new Rotation2d();
    double shooterPivotRotations =
        superstructure != null && superstructure.getArms().shooterPivot != null
            ? superstructure.getArms().shooterPivot.getPosition()
            : 0.0;

    boolean simTerrainEnabled = GlobalConstants.MODE == RobotMode.SIM;
    Pose3d terrainAdjustedRobotPose3d =
        simTerrainEnabled
            ? Rebuilt2026FieldModel.terrainAdjustedRobotPose(robotPose)
            : new Pose3d(robotPose);
    Pose3d authoritativeRobotPose3d =
        simTerrainEnabled && authoritativeRobotPose3dSupplier != null
            ? authoritativeRobotPose3dSupplier.get()
            : null;
    boolean usingAuthoritativeRobotPose = isValidPose3d(authoritativeRobotPose3d);
    Pose3d robotPose3d =
        usingAuthoritativeRobotPose ? authoritativeRobotPose3d : terrainAdjustedRobotPose3d;
    Pose3d shooterExitPose3d =
        ShooterComponentPublisher.createExitPose(
            robotPose3d, turretYaw, shooterPivotRotations, shotSimulationConfig);
    Pose3d turretPose3d =
        TurretComponentPublisher.createPose3d(robotPose3d, turretYaw, shotSimulationConfig);
    Pose3d shooterPivotPose3d =
        ShooterComponentPublisher.createPivotPose(
            robotPose3d, turretYaw, shooterPivotRotations, shotSimulationConfig);

    Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose3d", robotPose3d);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3dAuthoritative",
        usingAuthoritativeRobotPose ? authoritativeRobotPose3d : new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3dTerrainAdjusted", terrainAdjustedRobotPose3d);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3dUsesAuthoritativeSource", usingAuthoritativeRobotPose);
    Logger.recordOutput(
        "FieldSimulation/TurretPose",
        TurretComponentPublisher.createPose2d(robotPose, turretYaw, shotSimulationConfig));
    Logger.recordOutput("FieldSimulation/TurretComponentPose3d", new Pose3d[] {turretPose3d});
    Logger.recordOutput(
        "FieldSimulation/ShooterPivotComponentPose3d", new Pose3d[] {shooterPivotPose3d});
    Logger.recordOutput("FieldSimulation/ShooterExitPose3d", new Pose3d[] {shooterExitPose3d});
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPoses",
        new Pose3d[] {turretPose3d, shooterPivotPose3d, shooterExitPose3d});
    Logger.recordOutput(
        "FieldSimulation/FieldMarkers3d",
        simTerrainEnabled ? Rebuilt2026FieldModel.staticFieldMarkers() : new Pose3d[] {});
    Logger.recordOutput(
        "FieldSimulation/BumpHeightMeters",
        simTerrainEnabled
            ? Rebuilt2026FieldModel.bumpHeightMeters(robotPose.getTranslation())
            : 0.0);
    Logger.recordOutput(
        "FieldSimulation/BumpPitchRadians",
        simTerrainEnabled
            ? Rebuilt2026FieldModel.bumpPitchRadians(robotPose.getTranslation())
            : 0.0);

    SimulatedShot predictedShot = null;
    SuperstructureOutcome outcome =
        superstructure != null ? superstructure.getLatestOutcome() : null;
    if (superstructure != null && superstructure.isInAllianceZone() && outcome != null) {
      Translation2d turretTarget = outcome.turretTarget();
      Rotation2d commandedTurretYaw =
          turretTarget != null
              ? Rotation2d.fromRadians(TurretUtil.turretAngleToTarget(robotPose, turretTarget))
              : turretYaw;
      double predictedPivotRotations =
          outcome.shooterPivotManual() ? outcome.shooterPivotPosition() : shooterPivotRotations;
      predictedShot =
          shotSimulator
              .solveHubShot(
                  robotPose,
                  sanitize(drive != null ? drive.getFieldVelocity() : null),
                  commandedTurretYaw,
                  predictedPivotRotations,
                  outcome.shooterTargetVelocityRpm(),
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
          predictedShot.closestApproachErrorMeters(),
          predictedShot.timeOfFlightSeconds());
    } else {
      clearPredictionOutputs();
      shotReviewEvents.recordShotPrediction(false, Double.NaN, Double.NaN);
    }

    boolean released = false;
    ProjectileSnapshot spawnedProjectile = null;
    if (superstructure != null) {
      boolean shooterReady =
          superstructure.getRollers().shooter == null
              || superstructure.getRollers().shooter.isAtGoal();
      boolean armed =
          predictedShot != null
              && superstructure.hasBall()
              && shooterReady
              && outcome.indexerGoal() == IndexerGoal.FORWARD
              && outcome.shooterTargetVelocityRpm() > 1.0
              && (outcome.state() == SuperState.SHOOTING
                  || outcome.state() == SuperState.SHOOT_INTAKE
                  || outcome.state() == SuperState.FERRYING);
      released = shotReleaseDetector.update(armed);
      if (released) {
        spawnedProjectile = projectileManager.spawn(predictedShot);
      }
    }

    List<ProjectileSnapshot> completedProjectiles =
        projectileManager.update(AlignConstants.LOOP_PERIOD_SEC);
    List<ProjectileSnapshot> activeProjectiles = projectileManager.activeProjectileSnapshots();
    shotTelemetryAccumulator.update(
        Timer.getFPGATimestamp(),
        released,
        predictedShot,
        spawnedProjectile,
        activeProjectiles,
        completedProjectiles);
    ShotTelemetrySnapshot shotTelemetry = shotTelemetryAccumulator.snapshot();
    gamePiecePublisher.publishActiveProjectiles(projectileManager.activeProjectilePoses());
    Logger.recordOutput(
        "FieldSimulation/ObservedShotArc", poseArray(shotTelemetry.visibleObservedArcSamples()));
    Logger.recordOutput("ShotValidation/ReleaseCount", shotTelemetry.releaseCount());
    Logger.recordOutput("ShotValidation/CompletedCount", shotTelemetry.completedCount());
    Logger.recordOutput("ShotValidation/ActiveProjectileCount", activeProjectiles.size());
    Logger.recordOutput(
        "ShotValidation/LastLaunchSpeedMetersPerSecond",
        shotTelemetry.lastReleasedShot() == null
            ? Double.NaN
            : shotTelemetry.lastReleasedShot().launchSpeedMetersPerSecond());
    Logger.recordOutput(
        "ShotValidation/LastLaunchAngleDegrees",
        shotTelemetry.lastReleasedShot() == null
            ? Double.NaN
            : Math.toDegrees(shotTelemetry.lastReleasedShot().launchAngleRadians()));
    Logger.recordOutput(
        "ShotValidation/LastPredictionFeasible",
        shotTelemetry.lastReleasedShot() != null
            && shotTelemetry.lastReleasedShot().predictionFeasible());
    Logger.recordOutput(
        "ShotValidation/LastApexHeightMeters",
        shotTelemetry.lastReleasedShot() == null
            ? Double.NaN
            : shotTelemetry.lastReleasedShot().apexHeightMeters());
    Logger.recordOutput(
        "ShotValidation/LastTimeOfFlightSeconds",
        shotTelemetry.lastCompletedShot() == null
            ? Double.NaN
            : shotTelemetry.lastCompletedShot().timeOfFlightSeconds());
    shotReviewEvents.recordShotRelease(released);
    Logger.recordOutput("FieldSimulation/ProjectileCount", projectileManager.activeCount());
    Logger.recordOutput("FieldSimulation/ProjectileSpawnCount", projectileManager.spawnedCount());
    lastRenderTelemetry =
        new RenderTelemetry(
            robotPose3d,
            authoritativeRobotPose3d,
            terrainAdjustedRobotPose3d,
            usingAuthoritativeRobotPose);
  }

  public void reset() {
    projectileManager.clear();
    shotReleaseDetector.reset();
    shotTelemetryAccumulator.reset();
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

  private static boolean isValidPose3d(Pose3d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(pose.getRotation().getX())
        && Double.isFinite(pose.getRotation().getY())
        && Double.isFinite(pose.getRotation().getZ());
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

  public RenderTelemetry getRenderTelemetry() {
    return lastRenderTelemetry;
  }

  public ShotTelemetrySnapshot getShotTelemetrySnapshot() {
    return shotTelemetryAccumulator.snapshot();
  }

  private static Pose3d[] poseArray(List<ShotTelemetrySample> samples) {
    if (samples == null || samples.isEmpty()) {
      return new Pose3d[] {};
    }
    return samples.stream()
        .map(
            sample ->
                new Pose3d(sample.xMeters(), sample.yMeters(), sample.zMeters(), new Rotation3d()))
        .toArray(Pose3d[]::new);
  }

  public record RenderTelemetry(
      Pose3d publishedRobotPose3d,
      Pose3d authoritativeRobotPose3d,
      Pose3d terrainAdjustedRobotPose3d,
      boolean usingAuthoritativeRobotPose) {}

  public record ShotTelemetrySnapshot(
      int releaseCount,
      int completedCount,
      ShotTelemetryRecord activeShot,
      ShotTelemetryRecord lastReleasedShot,
      ShotTelemetryRecord lastCompletedShot,
      List<ShotTelemetrySample> visibleObservedArcSamples) {}

  public record ShotTelemetryRecord(
      int projectileId,
      double releaseTimestampSeconds,
      Pose3d releasePose,
      Translation3d releaseVelocityMetersPerSecond,
      double launchSpeedMetersPerSecond,
      double launchAngleRadians,
      boolean predictionFeasible,
      double predictionClosestApproachErrorMeters,
      double predictionTimeOfFlightSeconds,
      boolean clearsTop,
      boolean descendsIntoBottom,
      double topClearanceMeters,
      double bottomEntryErrorMeters,
      Pose3d topClearancePose,
      Pose3d bottomEntryPose,
      List<ShotTelemetrySample> predictedArcSamples,
      List<ShotTelemetrySample> observedArcSamples,
      double apexHeightMeters,
      boolean completed,
      boolean contactOccurred,
      boolean bounceOccurred,
      double timeOfFlightSeconds,
      Pose3d impactPose,
      Translation3d impactVelocityMetersPerSecond,
      double impactSpeedMetersPerSecond) {}

  public record ShotTelemetrySample(
      double timestampSeconds,
      double elapsedSeconds,
      double xMeters,
      double yMeters,
      double zMeters,
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double vzMetersPerSecond,
      double speedMetersPerSecond) {}

  private static final class ShotTelemetryAccumulator {
    private final Map<Integer, MutableShotTelemetry> activeShots = new LinkedHashMap<>();
    private ShotTelemetryRecord lastReleasedShot = null;
    private ShotTelemetryRecord lastCompletedShot = null;
    private int releaseCount = 0;
    private int completedCount = 0;

    void update(
        double timestampSeconds,
        boolean released,
        SimulatedShot releasedShot,
        ProjectileSnapshot spawnedProjectile,
        List<ProjectileSnapshot> activeProjectiles,
        List<ProjectileSnapshot> completedProjectiles) {
      if (released && releasedShot != null && spawnedProjectile != null) {
        MutableShotTelemetry shot =
            MutableShotTelemetry.fromRelease(timestampSeconds, releasedShot, spawnedProjectile);
        activeShots.put(spawnedProjectile.id(), shot);
        releaseCount++;
        lastReleasedShot = shot.snapshot(false);
      }

      if (activeProjectiles != null) {
        for (ProjectileSnapshot projectile : activeProjectiles) {
          MutableShotTelemetry shot = activeShots.get(projectile.id());
          if (shot != null) {
            shot.sample(timestampSeconds, projectile);
            if (lastReleasedShot != null && lastReleasedShot.projectileId() == projectile.id()) {
              lastReleasedShot = shot.snapshot(false);
            }
          }
        }
      }

      if (completedProjectiles != null) {
        for (ProjectileSnapshot projectile : completedProjectiles) {
          MutableShotTelemetry shot = activeShots.remove(projectile.id());
          if (shot == null) {
            continue;
          }
          shot.complete(timestampSeconds, projectile);
          completedCount++;
          lastCompletedShot = shot.snapshot(true);
          lastReleasedShot = lastCompletedShot;
        }
      }
    }

    ShotTelemetrySnapshot snapshot() {
      ShotTelemetryRecord activeShot =
          activeShots.isEmpty() ? null : activeShots.values().iterator().next().snapshot(false);
      List<ShotTelemetrySample> visibleArc =
          activeShot != null
              ? activeShot.observedArcSamples()
              : lastCompletedShot != null ? lastCompletedShot.observedArcSamples() : List.of();
      return new ShotTelemetrySnapshot(
          releaseCount,
          completedCount,
          activeShot,
          lastReleasedShot,
          lastCompletedShot,
          visibleArc);
    }

    void reset() {
      activeShots.clear();
      lastReleasedShot = null;
      lastCompletedShot = null;
      releaseCount = 0;
      completedCount = 0;
    }
  }

  private static final class MutableShotTelemetry {
    private final int projectileId;
    private final double releaseTimestampSeconds;
    private final Pose3d releasePose;
    private final Translation3d releaseVelocityMetersPerSecond;
    private final double launchSpeedMetersPerSecond;
    private final double launchAngleRadians;
    private final boolean predictionFeasible;
    private final double predictionClosestApproachErrorMeters;
    private final double predictionTimeOfFlightSeconds;
    private final boolean clearsTop;
    private final boolean descendsIntoBottom;
    private final double topClearanceMeters;
    private final double bottomEntryErrorMeters;
    private final Pose3d topClearancePose;
    private final Pose3d bottomEntryPose;
    private final List<ShotTelemetrySample> predictedArcSamples;
    private final List<ShotTelemetrySample> observedArcSamples = new ArrayList<>();
    private boolean completed = false;
    private boolean contactOccurred = false;
    private boolean bounceOccurred = false;
    private double timeOfFlightSeconds = Double.NaN;
    private Pose3d impactPose = null;
    private Translation3d impactVelocityMetersPerSecond = null;
    private double impactSpeedMetersPerSecond = Double.NaN;

    private MutableShotTelemetry(
        int projectileId,
        double releaseTimestampSeconds,
        Pose3d releasePose,
        Translation3d releaseVelocityMetersPerSecond,
        boolean predictionFeasible,
        double predictionClosestApproachErrorMeters,
        double predictionTimeOfFlightSeconds,
        boolean clearsTop,
        boolean descendsIntoBottom,
        double topClearanceMeters,
        double bottomEntryErrorMeters,
        Pose3d topClearancePose,
        Pose3d bottomEntryPose,
        List<ShotTelemetrySample> predictedArcSamples) {
      this.projectileId = projectileId;
      this.releaseTimestampSeconds = releaseTimestampSeconds;
      this.releasePose = releasePose;
      this.releaseVelocityMetersPerSecond = releaseVelocityMetersPerSecond;
      this.launchSpeedMetersPerSecond = norm(releaseVelocityMetersPerSecond);
      double horizontalSpeed =
          Math.hypot(releaseVelocityMetersPerSecond.getX(), releaseVelocityMetersPerSecond.getY());
      this.launchAngleRadians = Math.atan2(releaseVelocityMetersPerSecond.getZ(), horizontalSpeed);
      this.predictionFeasible = predictionFeasible;
      this.predictionClosestApproachErrorMeters = predictionClosestApproachErrorMeters;
      this.predictionTimeOfFlightSeconds = predictionTimeOfFlightSeconds;
      this.clearsTop = clearsTop;
      this.descendsIntoBottom = descendsIntoBottom;
      this.topClearanceMeters = topClearanceMeters;
      this.bottomEntryErrorMeters = bottomEntryErrorMeters;
      this.topClearancePose = topClearancePose;
      this.bottomEntryPose = bottomEntryPose;
      this.predictedArcSamples = predictedArcSamples;
    }

    static MutableShotTelemetry fromRelease(
        double timestampSeconds, SimulatedShot shot, ProjectileSnapshot spawnedProjectile) {
      MutableShotTelemetry telemetry =
          new MutableShotTelemetry(
              spawnedProjectile.id(),
              timestampSeconds,
              shot.releasePose(),
              shot.initialVelocityMetersPerSecond(),
              shot.feasible(),
              shot.closestApproachErrorMeters(),
              shot.timeOfFlightSeconds(),
              shot.clearsTop(),
              shot.descendsIntoBottom(),
              shot.topClearanceMeters(),
              shot.bottomEntryErrorMeters(),
              shot.topClearancePose(),
              shot.bottomEntryPose(),
              predictedSamples(timestampSeconds, shot.predictedSamplePoses()));
      telemetry.sample(timestampSeconds, spawnedProjectile);
      return telemetry;
    }

    void sample(double timestampSeconds, ProjectileSnapshot projectile) {
      observedArcSamples.add(
          new ShotTelemetrySample(
              timestampSeconds,
              projectile.ageSeconds(),
              projectile.positionMeters().getX(),
              projectile.positionMeters().getY(),
              projectile.positionMeters().getZ(),
              projectile.velocityMetersPerSecond().getX(),
              projectile.velocityMetersPerSecond().getY(),
              projectile.velocityMetersPerSecond().getZ(),
              norm(projectile.velocityMetersPerSecond())));
    }

    void complete(double timestampSeconds, ProjectileSnapshot projectile) {
      if (projectile.impactPositionMeters() != null) {
        observedArcSamples.add(
            new ShotTelemetrySample(
                Double.isFinite(projectile.impactTimestampSec())
                    ? projectile.impactTimestampSec()
                    : timestampSeconds,
                projectile.ageSeconds(),
                projectile.impactPositionMeters().getX(),
                projectile.impactPositionMeters().getY(),
                projectile.impactPositionMeters().getZ(),
                projectile.impactVelocityMetersPerSecond() == null
                    ? projectile.velocityMetersPerSecond().getX()
                    : projectile.impactVelocityMetersPerSecond().getX(),
                projectile.impactVelocityMetersPerSecond() == null
                    ? projectile.velocityMetersPerSecond().getY()
                    : projectile.impactVelocityMetersPerSecond().getY(),
                projectile.impactVelocityMetersPerSecond() == null
                    ? projectile.velocityMetersPerSecond().getZ()
                    : projectile.impactVelocityMetersPerSecond().getZ(),
                norm(
                    projectile.impactVelocityMetersPerSecond() == null
                        ? projectile.velocityMetersPerSecond()
                        : projectile.impactVelocityMetersPerSecond())));
      }
      completed = true;
      contactOccurred = projectile.impacted();
      bounceOccurred = false;
      timeOfFlightSeconds = projectile.ageSeconds();
      impactPose =
          projectile.impactPositionMeters() == null
              ? new Pose3d(projectile.positionMeters(), new Rotation3d())
              : new Pose3d(projectile.impactPositionMeters(), new Rotation3d());
      impactVelocityMetersPerSecond =
          projectile.impactVelocityMetersPerSecond() == null
              ? projectile.velocityMetersPerSecond()
              : projectile.impactVelocityMetersPerSecond();
      impactSpeedMetersPerSecond = norm(impactVelocityMetersPerSecond);
    }

    ShotTelemetryRecord snapshot(boolean forceCompleted) {
      return new ShotTelemetryRecord(
          projectileId,
          releaseTimestampSeconds,
          releasePose,
          releaseVelocityMetersPerSecond,
          launchSpeedMetersPerSecond,
          launchAngleRadians,
          predictionFeasible,
          predictionClosestApproachErrorMeters,
          predictionTimeOfFlightSeconds,
          clearsTop,
          descendsIntoBottom,
          topClearanceMeters,
          bottomEntryErrorMeters,
          topClearancePose,
          bottomEntryPose,
          List.copyOf(predictedArcSamples),
          List.copyOf(observedArcSamples),
          observedArcSamples.stream()
              .mapToDouble(ShotTelemetrySample::zMeters)
              .max()
              .orElse(Double.NaN),
          completed || forceCompleted,
          contactOccurred,
          bounceOccurred,
          timeOfFlightSeconds,
          impactPose,
          impactVelocityMetersPerSecond,
          impactSpeedMetersPerSecond);
    }

    private static List<ShotTelemetrySample> predictedSamples(
        double releaseTimestampSeconds, Pose3d[] predictedPoses) {
      if (predictedPoses == null || predictedPoses.length == 0) {
        return List.of();
      }
      List<ShotTelemetrySample> samples = new ArrayList<>();
      for (int i = 0; i < predictedPoses.length; i++) {
        Pose3d pose = predictedPoses[i];
        double elapsedSeconds = i * 0.02;
        samples.add(
            new ShotTelemetrySample(
                releaseTimestampSeconds + elapsedSeconds,
                elapsedSeconds,
                pose.getX(),
                pose.getY(),
                pose.getZ(),
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN));
      }
      return samples;
    }

    private static double norm(Translation3d vector) {
      return vector == null ? Double.NaN : vector.getNorm();
    }
  }
}
