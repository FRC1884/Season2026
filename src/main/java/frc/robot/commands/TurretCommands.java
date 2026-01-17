package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.TurretUtil;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class TurretCommands {
  private TurretCommands() {}

  public static Command turretToAngle(TurretSubsystem turret, DoubleSupplier angleRadSupplier) {
    return Commands.run(() -> turret.setGoalRad(angleRadSupplier.getAsDouble()), turret);
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
          Function<Pose2d, Optional<Translation2d>> targetSupplier,
          Supplier<ChassisSpeeds> robotVelocitySupplier,
          Optional<Alliance> ally,
          Map<Double, Double> lookupTable )
{
      return Commands.run(
          () -> {
              Alliance alliance = ally.get();
              Translation2d target = alliance == Alliance.Blue ? new Translation2d(4.63, 4.03) : 
              new Translation2d(11.9, 4.03);
              
              ChassisSpeeds speeds = robotVelocitySupplier.get();
              Pose2d currentPose = robotPoseSupplier.get();
              Translation2d currentTranslation = currentPose.getTranslation();
              double dist = target.getDistance(currentTranslation);
              double shotTime = ShooterCommands.find(dist);
              

              double futureX = currentTranslation.getX()+(speeds.vxMetersPerSecond)*shotTime;
              double futureY = currentTranslation.getY()+(speeds.vyMetersPerSecond)*shotTime;
              Translation2d futureTranslation = new Translation2d(futureX, futureY);
              Translation2d newTarget = target.minus(futureTranslation);
          });

          //TODO: Something about a method that returns a command???
  }
}
