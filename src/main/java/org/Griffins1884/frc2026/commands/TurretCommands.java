package org.Griffins1884.frc2026.commands;

import static org.Griffins1884.frc2026.commands.AlignConstants.*;
import static org.Griffins1884.frc2026.commands.ShooterCommands.getShooterRpm;
import static org.ironmaple.simulation.gamepieces.GamePieceProjectile.GRAVITY;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;

public final class TurretCommands {
  private static final int MAX_LEAD_ITERATIONS = 10;
  private static final double MIN_VALID_TOF_SECONDS = 1e-4;

  private TurretCommands() {}

  public static Command turretToZero(TurretSubsystem turret) {
    return Commands.runOnce(() -> turret.setGoalRad(0.0), turret);
  }

  public static Command autoAimToTarget(
      TurretSubsystem turret,
      Supplier<Pose2d> robotPoseSupplier,
      Function<Pose2d, Optional<Translation2d>> targetSupplier) {
    return Commands.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();
          Optional<Translation2d> target = targetSupplier.apply(robotPose);
          if (GlobalConstants.isDebugMode()) {
            Logger.recordOutput("Turret/AutoAim/HasTarget", target.isPresent());
          }
          if (target.isEmpty()) {
            turret.setGoalRad(turret.getPositionRad());
            if (GlobalConstants.isDebugMode()) {
              Logger.recordOutput("Turret/AutoAim/GoalRad", turret.getGoalRad());
            }
            return;
          }
          double goalRad = TurretUtil.turretAngleToTarget(robotPose, target.get());
          turret.setGoalRad(goalRad);
          if (GlobalConstants.isDebugMode()) {
            Logger.recordOutput("Turret/AutoAim/Target", target.get());
            Logger.recordOutput("Turret/AutoAim/GoalRad", goalRad);
          }
        },
        turret);
  }

  public static Translation2d shootingWhileMoving(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<Translation2d> fieldVelocitySupplier,
      Supplier<Translation2d> fieldAccelerationSupplier) {
    return shootingWhileMoving(
        robotPoseSupplier,
        targetSupplier,
        fieldVelocitySupplier,
        fieldAccelerationSupplier,
        TurretCommands::estimateShotTimeSeconds);
  }

  static Translation2d shootingWhileMoving(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<Translation2d> fieldVelocitySupplier,
      Supplier<Translation2d> fieldAccelerationSupplier,
      DoubleUnaryOperator shotTimeEstimator) {
    Pose2d currentPose = robotPoseSupplier.get();
    Translation2d target = targetSupplier.get();
    if (currentPose == null || target == null || shotTimeEstimator == null) {
      return new Translation2d();
    }
    Translation2d currentTranslation = currentPose.getTranslation();
    Rotation2d angle = new Rotation2d();

    Translation2d fieldVelocity =
        sanitizeVector(fieldVelocitySupplier != null ? fieldVelocitySupplier.get() : null);
    Translation2d fieldAcceleration =
        sanitizeVector(fieldAccelerationSupplier != null ? fieldAccelerationSupplier.get() : null);
    double latency = sanitizeNonNegative(TURRET_BASE_LATENCY_SECONDS.getAsDouble());
    double tofToleranceFraction =
        Math.max(MIN_VALID_TOF_SECONDS, sanitizeNonNegative(ALIGN_TOF_TOLERANCE_FRACTION.get()));

    Translation2d robotTranslateExit =
        fieldVelocity.times(latency).plus(fieldAcceleration.times(0.5 * latency * latency));
    Translation2d robotVelocityExit = fieldVelocity.plus(fieldAcceleration.times(latency));
    Translation2d baseVectorFromExit = target.minus(currentTranslation.plus(robotTranslateExit));

    Translation2d aimVector = baseVectorFromExit;
    double dist = baseVectorFromExit.getNorm();
    double tof = shotTimeEstimator.applyAsDouble(dist);
    if (!Double.isFinite(tof) || tof <= MIN_VALID_TOF_SECONDS) {
      Logger.recordOutput("Turret/ShootingWhileMoving/Fallback", true);
      return target;
    }
    double tEff = tof + latency;
    int iterationsUsed = 0;

    for (int i = 0; i < MAX_LEAD_ITERATIONS; i++) {
      iterationsUsed = i + 1;
      Translation2d lead = robotVelocityExit.times(tof);
      aimVector = baseVectorFromExit.minus(lead);

      double newDist = aimVector.getNorm();
      double newTOF = shotTimeEstimator.applyAsDouble(newDist);
      if (!Double.isFinite(newTOF) || newTOF <= MIN_VALID_TOF_SECONDS) {
        break;
      }

      double oldTEff = tEff;
      double newTEff = newTOF + latency;

      double check = Math.abs(newTEff - oldTEff) / Math.max(oldTEff, 1e-6);
      dist = newDist;
      tof = newTOF;
      tEff = newTEff;
      angle = new Rotation2d(aimVector.getX(), aimVector.getY());

      if (check <= tofToleranceFraction) {
        break;
      }
    }

    // Convert back into field coordinates from the predicted release pose.
    Translation2d aimPoint = currentTranslation.plus(robotTranslateExit).plus(aimVector);
    Logger.recordOutput("Turret/ShootingWhileMoving/Fallback", false);
    Logger.recordOutput("Turret/ShootingWhileMoving/LatencySec", latency);
    Logger.recordOutput("Turret/ShootingWhileMoving/IterationsUsed", iterationsUsed);
    Logger.recordOutput("Turret/ShootingWhileMoving/EffectiveTimeSec", tEff);
    Logger.recordOutput("Turret/ShootingWhileMoving/ShotTimeSec", tof);
    Logger.recordOutput("Turret/ShootingWhileMoving/DistanceMeters", dist);
    Logger.recordOutput("Turret/ShootingWhileMoving/AimPoint", aimPoint);
    Logger.recordOutput(
        "Turret/ShootingWhileMoving/LeadOffsetMeters", target.minus(aimPoint).getNorm());
    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Turret/AutoAim/FutureTarget", aimPoint);
      Logger.recordOutput("Turret/AutoAim/AngleToTarget", angle);
    }
    return aimPoint;
  }

  private static Translation2d sanitizeVector(Translation2d vector) {
    if (vector == null) {
      return new Translation2d();
    }
    double x = vector.getX();
    double y = vector.getY();
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      return new Translation2d();
    }
    return vector;
  }

  private static double sanitizeNonNegative(double value) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return Math.max(0.0, value);
  }

  public static ShooterCommands.ShotTimeEstimate estimateShotTimeDetailed(
      double distanceMeters,
      double hoodAngleRad,
      double shooterExitHeightMeters,
      double targetHeightMeters,
      double wheelRpm,
      double wheelRadiusMeters,
      double gearRatio,
      double slipFactor) {
    if (distanceMeters <= 0.0) {
      return new ShooterCommands.ShotTimeEstimate(0.0, 0.0, shooterExitHeightMeters, 0.0, false);
    }
    double exitVelocity =
        (wheelRpm / 60.0) * (2.0 * Math.PI) * wheelRadiusMeters * gearRatio * slipFactor;
    double cos = Math.cos(hoodAngleRad);
    if (Math.abs(cos) < 1e-6 || exitVelocity <= 1e-6) {
      return new ShooterCommands.ShotTimeEstimate(
          0.0, exitVelocity, shooterExitHeightMeters, 0.0, false);
    }
    double timeSeconds = distanceMeters / (exitVelocity * cos);
    double predictedHeight =
        shooterExitHeightMeters
            + exitVelocity * Math.sin(hoodAngleRad) * timeSeconds
            - 0.5 * GRAVITY * timeSeconds * timeSeconds;
    double heightError = targetHeightMeters - predictedHeight;
    boolean feasible = !Double.isNaN(timeSeconds) && timeSeconds > 0.0;
    return new ShooterCommands.ShotTimeEstimate(
        timeSeconds, exitVelocity, predictedHeight, heightError, feasible);
  }

  public static double estimateShotTimeSeconds(double distanceMeters) {

    ShooterCommands.ShotTimeEstimate estimate =
        estimateShotTimeDetailed(
            distanceMeters,
            (Math.PI / 2) - ShooterCommands.getPivotAngleRad(distanceMeters),
            ShooterConstants.EXIT_HEIGHT_METERS,
            ShooterConstants.TARGET_HEIGHT_METERS,
            getShooterRpm(distanceMeters),
            ShooterConstants.FLYWHEEL_RADIUS_METERS,
            ShooterConstants.FLYWHEEL_GEAR_RATIO,
            ShooterConstants.SLIP_FACTOR.get());
    return estimate.timeSeconds();
  }
}
