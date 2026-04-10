package org.Griffins1884.frc2026.simulation.maple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.TerrainModel;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.integration.FieldMarkerSample;

/** Terrain/marker helpers for the local 2026 field model used in simulation and AdvantageScope. */
public final class Rebuilt2026FieldModel implements TerrainModel {
  private static final Season2026FieldContactModel CONTACT_MODEL =
      Season2026FieldContactModel.INSTANCE;
  public static final ChassisFootprint CHASSIS_FOOTPRINT = Season2026RobotProfile.DEFAULT_FOOTPRINT;
  public static final ChassisMassProperties CHASSIS_MASS_PROPERTIES =
      Season2026RobotProfile.DEFAULT_CHASSIS_MASS_PROPERTIES;

  public static final Rebuilt2026FieldModel INSTANCE = new Rebuilt2026FieldModel();

  private Rebuilt2026FieldModel() {}

  public static Season2026FieldContactModel contactModel() {
    return CONTACT_MODEL;
  }

  public static void setValidationFlatTerrainOverride(boolean enabled) {
    Season2026FieldContactModel.setValidationFlatTerrainOverride(enabled);
  }

  public static Pose3d terrainAdjustedRobotPose(Pose2d robotPose) {
    return INSTANCE.sample(robotPose).pose3d();
  }

  @Override
  public TerrainSample sample(Pose2d robotPose) {
    return CONTACT_MODEL.sample(robotPose);
  }

  public static Pose3d[] staticFieldMarkers() {
    return Arrays.stream(CONTACT_MODEL.getFieldMarkers())
        .map(FieldMarkerSample::pose)
        .toArray(Pose3d[]::new);
  }

  public static double bumpHeightMeters(Translation2d fieldTranslation) {
    return CONTACT_MODEL.sample(new Pose2d(fieldTranslation, new Rotation2d())).heightMeters();
  }

  public static double bumpPitchRadians(Translation2d fieldTranslation) {
    return CONTACT_MODEL.sample(new Pose2d(fieldTranslation, new Rotation2d())).pitchRadians();
  }

  public static double bumpRollRadians(Translation2d fieldTranslation) {
    return CONTACT_MODEL.sample(new Pose2d(fieldTranslation, new Rotation2d())).rollRadians();
  }
}
