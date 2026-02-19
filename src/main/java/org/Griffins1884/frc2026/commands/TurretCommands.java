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
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;

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
    double dist = target.getDistance(currentTranslation);
    double shotTime = ShooterCommands.estimateShotTimeSeconds(dist, ShooterCommands.getBestAngle(dist));

    Translation2d fieldVelocity =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(currentPose.getRotation());
    Translation2d futureTranslation =
        currentTranslation.plus(
            new Translation2d(fieldVelocity.getX() * shotTime, fieldVelocity.getY() * shotTime));
    Rotation2d futureRotation =
        Rotation2d.fromRadians(
            currentPose.getRotation().getRadians() + speeds.omegaRadiansPerSecond * shotTime);
    Pose2d futurePose = new Pose2d(futureTranslation, futureRotation);
    Logger.recordOutput("Turret/AutoAim/ShotTime", shotTime);
    Logger.recordOutput("Turret/AutoAim/Distance", dist);
    Logger.recordOutput("Turret/AutoAim/FuturePose", futurePose);
    return TurretUtil.turretAngleToTarget(futurePose, target);
  }
}
