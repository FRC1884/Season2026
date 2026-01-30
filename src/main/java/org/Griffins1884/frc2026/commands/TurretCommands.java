package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;

public final class TurretCommands {
  private TurretCommands() {}

  // public static Command turretToAngle(TurretSubsystem turret, DoubleSupplier angleRadSupplier) {
  // return Commands.run(() -> turret.setGoalRad(angleRadSupplier.getAsDouble()), turret);
  // }

  public static Command turretToZero(TurretSubsystem turret) {
    return Commands.runOnce(() -> turret.setGoalRad(0.0), turret);
  }

  /** Command to rotate turret to specific angle with wrapping */
  public static Command turretToAngle(TurretSubsystem turret, DoubleSupplier angleRadSupplier) {
    return Commands.run(
        () -> {
          double targetAngle = angleRadSupplier.getAsDouble();
          double currentAngle = turret.getPositionRad();
          double wrappedAngle = wrapAngleToShortest(currentAngle, targetAngle);
          turret.setGoalRad(wrappedAngle);
        },
        turret);
  }

  /**
   * Calculates the shortest angular path between current and target angles Prevents turret from
   * rotating more than 180 degrees
   */
  private static double wrapAngleToShortest(double currentAngle, double targetAngle) {
    // Calculate the angular difference
    double diff = targetAngle - currentAngle;

    // Wrap to shortest path using atan2 for proper quadrant handling
    diff = Math.atan2(Math.sin(diff), Math.cos(diff));

    return currentAngle + diff;
  }

  public static Command turretOpenLoop(TurretSubsystem turret, DoubleSupplier percentSupplier) {
    return Commands.runEnd(
        () -> turret.setOpenLoop(percentSupplier.getAsDouble()), turret::stopOpenLoop, turret);
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

  public static Command shootingWhileMoving(
      TurretSubsystem turret,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    ChassisSpeeds speeds = robotVelocitySupplier.get();
    Pose2d currentPose = robotPoseSupplier.get();
    Translation2d currentTranslation = currentPose.getTranslation();
    double dist = targetSupplier.get().getDistance(currentTranslation);
    double shotTime = ShooterCommands.find(dist);

    double futureX = currentTranslation.getX() + (speeds.vxMetersPerSecond) * shotTime;
    double futureY = currentTranslation.getY() + (speeds.vyMetersPerSecond) * shotTime;
    Translation2d futureTranslation = new Translation2d(futureX, futureY);
    Translation2d newTarget = targetSupplier.get().minus(futureTranslation);

    return autoAimToTarget(turret, robotPoseSupplier, pose -> Optional.of(newTarget));
  }
}
