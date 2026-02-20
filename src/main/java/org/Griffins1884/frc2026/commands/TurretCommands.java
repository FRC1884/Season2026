package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;

import static org.Griffins1884.frc2026.commands.AlignConstants.*;
import static org.Griffins1884.frc2026.commands.ShooterCommands.getRPM;
import static org.ironmaple.simulation.gamepieces.GamePieceProjectile.GRAVITY;

public final class TurretCommands {
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
          Logger.recordOutput("Turret/AutoAim/HasTarget", target.isPresent());
          if (target.isEmpty()) {
            turret.setGoalRad(turret.getPositionRad());
            Logger.recordOutput("Turret/AutoAim/GoalRad", turret.getGoalRad());
            return;
          }
          double goalRad = TurretUtil.turretAngleToTarget(robotPose, target.get());
          turret.setGoalRad(goalRad);
          Logger.recordOutput("Turret/AutoAim/Target", target.get());
          Logger.recordOutput("Turret/AutoAim/GoalRad", goalRad);
        },
        turret);
  }

  public static double shootingWhileMoving(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    Pose2d currentPose = robotPoseSupplier.get();
    Translation2d target = targetSupplier.get();
    if (currentPose == null || target == null) {
      return 0.0;
    }
    ChassisSpeeds speeds = robotVelocitySupplier.get();
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d baseVector = target.minus(currentTranslation);
    Translation2d aimVector = baseVector;
    double dist = baseVector.getNorm();
    double tof = estimateShotTimeSeconds(dist);
    if (!Double.isFinite(tof) || tof <= 1e-4) {
        return TurretUtil.turretAngleToTarget(currentPose, target);
    }
    Rotation2d angle = new Rotation2d(0);
    Translation2d fieldVelocity =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(currentPose.getRotation());
      double kV = TURRET_KV.getAsDouble();
      double kS = TURRET_KS.getAsDouble();

      for (int i = 0; i < 8; i++) {
          Translation2d lead = fieldVelocity.times(tof);
          Translation2d leadAdjusted = lead.times(kV + kS * dist);
          aimVector = baseVector.minus(leadAdjusted);

          double newDist = aimVector.getNorm();
          double newTOF = estimateShotTimeSeconds(newDist);
          if (!Double.isFinite(newTOF) || newTOF <= 1e-4) break;

          double check = Math.abs(newTOF - tof) / Math.max(tof, 1e-6);
          dist = newDist;
          angle = new Rotation2d(aimVector.getX(), aimVector.getY());

          if (check <= AlignConstants.ALIGN_TOF_TOLERANCE_FRACTION.getAsDouble()) {
              break;
          }

          tof = newTOF;
      }
    Translation2d aimPoint = currentPose.getTranslation().plus(aimVector);
    Logger.recordOutput("Turret/AutoAim/ShotTime", tof);
    Logger.recordOutput("Turret/AutoAim/Distance", dist);
    Logger.recordOutput("Turret/AutoAim/FutureTarget", aimPoint);
    Logger.recordOutput("Turret/AutoAim/AngleToTarget", angle);
    return TurretUtil.turretAngleToTarget(currentPose, aimPoint);
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
            return new ShooterCommands.ShotTimeEstimate(0.0, exitVelocity, shooterExitHeightMeters, 0.0, false);
        }
        double timeSeconds = distanceMeters / (exitVelocity * cos);
        double predictedHeight =
                shooterExitHeightMeters
                        + exitVelocity * Math.sin(hoodAngleRad) * timeSeconds
                        - 0.5 * GRAVITY * timeSeconds * timeSeconds;
        double heightError = targetHeightMeters - predictedHeight;
        boolean feasible = !Double.isNaN(timeSeconds) && timeSeconds > 0.0;
        return new ShooterCommands.ShotTimeEstimate(timeSeconds, exitVelocity, predictedHeight, heightError, feasible);
    }

    public static double estimateShotTimeSeconds(double distanceMeters) {

      ShooterCommands.ShotTimeEstimate estimate =
                estimateShotTimeDetailed(
                        distanceMeters,
                        ShooterCommands.getBestAngle(distanceMeters),
                        ShooterConstants.EXIT_HEIGHT_METERS,
                        ShooterConstants.TARGET_HEIGHT_METERS,
                        getRPM(distanceMeters),
                        ShooterConstants.FLYWHEEL_RADIUS_METERS,
                        ShooterConstants.FLYWHEEL_GEAR_RATIO,
                        ShooterConstants.SLIP_FACTOR.get());
        return estimate.timeSeconds();
    }
}
