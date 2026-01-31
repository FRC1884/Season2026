package org.Griffins1884.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.lang.reflect.Method;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretIOSim;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.junit.jupiter.api.Test;

class SuperstructureAimTurretTest {
  @Test
  void aimTurretAtSetsGoal() throws Exception {
    SwerveSubsystem drive = mock(SwerveSubsystem.class);
    Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d());
    when(drive.getPose()).thenReturn(pose);
    when(drive.getRobotRelativeSpeeds()).thenReturn(new ChassisSpeeds());

    Superstructure superstructure = new Superstructure(drive);
    TurretSubsystem turret = new TurretSubsystem(new TurretIOSim());
    superstructure.setTurret(turret);
    superstructure.setTurretExternalControl(false);

    Method aimMethod = Superstructure.class.getDeclaredMethod("aimTurretAt", Translation2d.class);
    aimMethod.setAccessible(true);

    Translation2d target = new Translation2d(0.0, 1.0);
    aimMethod.invoke(superstructure, target);

    double expected = TurretUtil.turretAngleToTarget(pose, target);
    assertEquals(expected, turret.getGoalRad(), 1e-9);
  }
}
