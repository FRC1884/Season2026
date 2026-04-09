package org.Griffins1884.frc2026.simulation.maple;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.simulation.engine.CollisionShape;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMaterial;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.engine.RigidBody;
import org.Griffins1884.frc2026.simulation.engine.RigidBodyType;

/** Local terrain model used by the rebuilt pure-physics simulation path. */
public final class Rebuilt2026FieldModel {
  public static final Rebuilt2026FieldModel INSTANCE = new Rebuilt2026FieldModel();

  private Rebuilt2026FieldModel() {}

  public static Pose3d terrainAdjustedRobotPose(Pose2d robotPose) {
    LocalTerrainSample sample = sample(robotPose);
    return new Pose3d(
        robotPose.getX(),
        robotPose.getY(),
        sample.heightMeters(),
        new Rotation3d(
            sample.rollRadians(), sample.pitchRadians(), robotPose.getRotation().getRadians()));
  }

  public static LocalTerrainSample sample(Pose2d robotPose) {
    Translation2d translation =
        robotPose != null ? robotPose.getTranslation() : new Translation2d();
    double x = translation.getX();
    double y = translation.getY();
    double[] centersX = {
      GlobalConstants.FieldConstants.LinesVertical.hubCenter,
      GlobalConstants.FieldConstants.LinesVertical.hubCenter,
      GlobalConstants.FieldConstants.fieldLength
          - GlobalConstants.FieldConstants.LinesVertical.hubCenter,
      GlobalConstants.FieldConstants.fieldLength
          - GlobalConstants.FieldConstants.LinesVertical.hubCenter
    };
    double[] centersY = {
      (GlobalConstants.FieldConstants.LinesHorizontal.leftBumpStart
              + GlobalConstants.FieldConstants.LinesHorizontal.leftBumpEnd)
          * 0.5,
      (GlobalConstants.FieldConstants.LinesHorizontal.rightBumpStart
              + GlobalConstants.FieldConstants.LinesHorizontal.rightBumpEnd)
          * 0.5,
      (GlobalConstants.FieldConstants.LinesHorizontal.leftBumpStart
              + GlobalConstants.FieldConstants.LinesHorizontal.leftBumpEnd)
          * 0.5,
      (GlobalConstants.FieldConstants.LinesHorizontal.rightBumpStart
              + GlobalConstants.FieldConstants.LinesHorizontal.rightBumpEnd)
          * 0.5
    };

    double sigmaX = GlobalConstants.FieldConstants.LeftBump.width / 4.0;
    double sigmaY = GlobalConstants.FieldConstants.LeftBump.depth / 4.0;
    double height = 0.0;
    double dHeightDx = 0.0;
    double dHeightDy = 0.0;

    for (int i = 0; i < centersX.length; i++) {
      double dx = x - centersX[i];
      double dy = y - centersY[i];
      double exponent = -0.5 * ((dx * dx) / (sigmaX * sigmaX) + (dy * dy) / (sigmaY * sigmaY));
      double contribution = GlobalConstants.FieldConstants.LeftBump.height * Math.exp(exponent);
      height += contribution;
      dHeightDx += contribution * (-dx / (sigmaX * sigmaX));
      dHeightDy += contribution * (-dy / (sigmaY * sigmaY));
    }

    double pitch = MathUtil.clamp(-Math.atan(dHeightDx), -0.35, 0.35);
    double roll = MathUtil.clamp(Math.atan(dHeightDy), -0.35, 0.35);
    return new LocalTerrainSample(height, pitch, roll);
  }

  public static Pose3d[] staticFieldMarkers() {
    return new Pose3d[] {
      new Pose3d(
          GlobalConstants.FieldConstants.LinesVertical.hubCenter,
          (GlobalConstants.FieldConstants.LinesHorizontal.leftBumpStart
                  + GlobalConstants.FieldConstants.LinesHorizontal.leftBumpEnd)
              * 0.5,
          GlobalConstants.FieldConstants.LeftBump.height,
          new Rotation3d()),
      new Pose3d(
          GlobalConstants.FieldConstants.LinesVertical.hubCenter,
          (GlobalConstants.FieldConstants.LinesHorizontal.rightBumpStart
                  + GlobalConstants.FieldConstants.LinesHorizontal.rightBumpEnd)
              * 0.5,
          GlobalConstants.FieldConstants.RightBump.height,
          new Rotation3d()),
      new Pose3d(GlobalConstants.FieldConstants.Hub.topCenterPoint, new Rotation3d())
    };
  }

  public static double bumpHeightMeters(Translation2d fieldTranslation) {
    return sample(new Pose2d(fieldTranslation, new Rotation2d())).heightMeters();
  }

  public static double bumpPitchRadians(Translation2d fieldTranslation) {
    return sample(new Pose2d(fieldTranslation, new Rotation2d())).pitchRadians();
  }

  public static double bumpRollRadians(Translation2d fieldTranslation) {
    return sample(new Pose2d(fieldTranslation, new Rotation2d())).rollRadians();
  }

  public static List<RigidBody> createStaticCollisionBodies() {
    PhysicsMaterial fieldMaterial = new PhysicsMaterial(0.9, 0.05, 0.02, 0.95);
    double fieldHalfLength = GlobalConstants.FieldConstants.fieldLength * 0.5;
    double fieldHalfWidth = GlobalConstants.FieldConstants.fieldWidth * 0.5;
    double leftCenterY =
        (GlobalConstants.FieldConstants.LinesHorizontal.leftBumpStart
                + GlobalConstants.FieldConstants.LinesHorizontal.leftBumpEnd)
            * 0.5;
    double rightCenterY =
        (GlobalConstants.FieldConstants.LinesHorizontal.rightBumpStart
                + GlobalConstants.FieldConstants.LinesHorizontal.rightBumpEnd)
            * 0.5;
    double hubCenter = GlobalConstants.FieldConstants.LinesVertical.hubCenter;
    return List.of(
        new RigidBody(
            1000,
            "field-ground",
            RigidBodyType.STATIC,
            CollisionShape.plane(Vec3.UNIT_Z, 0.0),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            Vec3.ZERO,
            Quat.IDENTITY),
        new RigidBody(
            1001,
            "left-bump-blue",
            RigidBodyType.STATIC,
            CollisionShape.box(
                GlobalConstants.FieldConstants.LeftBump.width * 0.5,
                GlobalConstants.FieldConstants.LeftBump.depth * 0.5,
                GlobalConstants.FieldConstants.LeftBump.height * 0.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(hubCenter, leftCenterY, GlobalConstants.FieldConstants.LeftBump.height * 0.5),
            Quat.IDENTITY),
        new RigidBody(
            1002,
            "right-bump-blue",
            RigidBodyType.STATIC,
            CollisionShape.box(
                GlobalConstants.FieldConstants.RightBump.width * 0.5,
                GlobalConstants.FieldConstants.RightBump.depth * 0.5,
                GlobalConstants.FieldConstants.RightBump.height * 0.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(
                hubCenter, rightCenterY, GlobalConstants.FieldConstants.RightBump.height * 0.5),
            Quat.IDENTITY),
        new RigidBody(
            1003,
            "left-bump-red",
            RigidBodyType.STATIC,
            CollisionShape.box(
                GlobalConstants.FieldConstants.LeftBump.width * 0.5,
                GlobalConstants.FieldConstants.LeftBump.depth * 0.5,
                GlobalConstants.FieldConstants.LeftBump.height * 0.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(
                GlobalConstants.FieldConstants.fieldLength - hubCenter,
                leftCenterY,
                GlobalConstants.FieldConstants.LeftBump.height * 0.5),
            Quat.IDENTITY),
        new RigidBody(
            1004,
            "right-bump-red",
            RigidBodyType.STATIC,
            CollisionShape.box(
                GlobalConstants.FieldConstants.RightBump.width * 0.5,
                GlobalConstants.FieldConstants.RightBump.depth * 0.5,
                GlobalConstants.FieldConstants.RightBump.height * 0.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(
                GlobalConstants.FieldConstants.fieldLength - hubCenter,
                rightCenterY,
                GlobalConstants.FieldConstants.RightBump.height * 0.5),
            Quat.IDENTITY),
        new RigidBody(
            1005,
            "field-left-wall",
            RigidBodyType.STATIC,
            CollisionShape.box(0.05, fieldHalfWidth, 1.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(-0.05, fieldHalfWidth, 1.5),
            Quat.IDENTITY),
        new RigidBody(
            1006,
            "field-right-wall",
            RigidBodyType.STATIC,
            CollisionShape.box(0.05, fieldHalfWidth, 1.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(GlobalConstants.FieldConstants.fieldLength + 0.05, fieldHalfWidth, 1.5),
            Quat.IDENTITY),
        new RigidBody(
            1007,
            "field-top-wall",
            RigidBodyType.STATIC,
            CollisionShape.box(fieldHalfLength, 0.05, 1.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(fieldHalfLength, GlobalConstants.FieldConstants.fieldWidth + 0.05, 1.5),
            Quat.IDENTITY),
        new RigidBody(
            1008,
            "field-bottom-wall",
            RigidBodyType.STATIC,
            CollisionShape.box(fieldHalfLength, 0.05, 1.5),
            fieldMaterial,
            0.0,
            Vec3.ZERO,
            new Vec3(fieldHalfLength, -0.05, 1.5),
            Quat.IDENTITY));
  }

  public record LocalTerrainSample(double heightMeters, double pitchRadians, double rollRadians) {}
}
