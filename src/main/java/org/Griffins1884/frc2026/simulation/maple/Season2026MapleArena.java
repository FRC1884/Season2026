package org.Griffins1884.frc2026.simulation.maple;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.Griffins1884.frc2026.GlobalConstants;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

/**
 * Season2026-owned Maple arena setup.
 *
 * <p>This replaces the direct dependency on the legacy GriffinSim rebuilt-season Maple arena while
 * preserving the same field collision layout for desktop simulation.
 */
public final class Season2026MapleArena extends SimulatedArena {
  public Season2026MapleArena() {
    this(true);
  }

  public Season2026MapleArena(boolean includeTraversalStructures) {
    super(new RebuiltFieldObstaclesMap(includeTraversalStructures));

    Settings settings = physicsWorld.getSettings();
    settings.setMinimumAtRestTime(0.02);
    physicsWorld.setSettings(settings);
  }

  @Override
  public void placeGamePiecesOnField() {
    // Season2026 controls game-piece spawning in robot-side validation flows.
  }

  public static final class RebuiltFieldObstaclesMap extends FieldMap {
    private static final double FIELD_X_MIN = 0.0;
    private static final double FIELD_X_MAX = GlobalConstants.FieldConstants.fieldLength;
    private static final double FIELD_Y_MIN = 0.0;
    private static final double FIELD_Y_MAX = GlobalConstants.FieldConstants.fieldWidth;

    private static final double INTERNAL_WALL_THICKNESS = Inches.of(2.0).in(Meters);
    private static final double TOWER_WIDTH_METERS = Inches.of(47.00).in(Meters);
    private static final double TOWER_DEPTH_METERS = Inches.of(40.00).in(Meters);
    private static final double BLUE_TOWER_FRONT_FACE_X_METERS = Inches.of(0.55).in(Meters);
    private static final double RED_TOWER_FRONT_FACE_X_METERS = Inches.of(649.57).in(Meters);
    private static final double TOWER_INNER_OPENING_WIDTH_METERS = Inches.of(32.250).in(Meters);
    private static final double TOWER_SIDE_WIDTH_METERS =
        (TOWER_WIDTH_METERS - TOWER_INNER_OPENING_WIDTH_METERS) * 0.5;

    RebuiltFieldObstaclesMap(boolean includeTraversalStructures) {
      addFieldBorder();
      addHubObstacles();
      addTrenchEdgeWalls();
      addTowerObstacles();

      if (!includeTraversalStructures) {
        addTraversalStructureBlocks();
      }
    }

    private void addFieldBorder() {
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MIN, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MAX, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MIN));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MAX), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));
    }

    private void addHubObstacles() {
      addHubPerimeter(blueHubFaceCenters());
      addHubPerimeter(redHubFaceCenters());
      addRectangularObstacle(
          Inches.of(35.0).in(Meters),
          Inches.of(35.0).in(Meters),
          new Pose2d(
              blueHubCenterX(), GlobalConstants.FieldConstants.fieldWidth / 2.0, new Rotation2d()));
      addRectangularObstacle(
          Inches.of(35.0).in(Meters),
          Inches.of(35.0).in(Meters),
          new Pose2d(
              redHubCenterX(), GlobalConstants.FieldConstants.fieldWidth / 2.0, new Rotation2d()));
    }

    private void addTrenchEdgeWalls() {
      addRectRegions(RebuiltFieldRegions.blueLeftTrenchEdgeRegions());
      addRectRegions(RebuiltFieldRegions.blueRightTrenchEdgeRegions());
      addRectRegions(RebuiltFieldRegions.redLeftTrenchEdgeRegions());
      addRectRegions(RebuiltFieldRegions.redRightTrenchEdgeRegions());
    }

    private void addTowerObstacles() {
      addTowerSideWalls(
          BLUE_TOWER_FRONT_FACE_X_METERS, GlobalConstants.FieldConstants.fieldWidth / 2.0, true);
      addTowerSideWalls(
          RED_TOWER_FRONT_FACE_X_METERS, GlobalConstants.FieldConstants.fieldWidth / 2.0, false);
    }

    private void addTraversalStructureBlocks() {
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(blueHubCenterX(), FIELD_Y_MAX - Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(blueHubCenterX(), Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(redHubCenterX(), FIELD_Y_MAX - Inches.of(60.0).in(Meters), new Rotation2d()));
      addRectangularObstacle(
          Inches.of(44.4).in(Meters),
          Inches.of(73.0).in(Meters),
          new Pose2d(redHubCenterX(), Inches.of(60.0).in(Meters), new Rotation2d()));
    }

    private void addTowerSideWalls(double frontFaceX, double centerY, boolean blueSide) {
      double towerCenterX =
          blueSide
              ? frontFaceX - (TOWER_DEPTH_METERS * 0.5)
              : frontFaceX + (TOWER_DEPTH_METERS * 0.5);
      double leftWallCenterY =
          centerY + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5) + (TOWER_SIDE_WIDTH_METERS * 0.5);
      double rightWallCenterY =
          centerY - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5) - (TOWER_SIDE_WIDTH_METERS * 0.5);

      addRectangularObstacle(
          TOWER_DEPTH_METERS,
          TOWER_SIDE_WIDTH_METERS,
          new Pose2d(towerCenterX, leftWallCenterY, new Rotation2d()));
      addRectangularObstacle(
          TOWER_DEPTH_METERS,
          TOWER_SIDE_WIDTH_METERS,
          new Pose2d(towerCenterX, rightWallCenterY, new Rotation2d()));

      double backFaceX =
          blueSide ? frontFaceX - TOWER_DEPTH_METERS : frontFaceX + TOWER_DEPTH_METERS;
      addRectangularObstacle(
          INTERNAL_WALL_THICKNESS,
          TOWER_WIDTH_METERS,
          new Pose2d(backFaceX, centerY, new Rotation2d()));
    }

    private void addRectRegion(RectRegion region) {
      addRectangularObstacle(
          region.maxX() - region.minX(),
          region.maxY() - region.minY(),
          new Pose2d(
              (region.minX() + region.maxX()) * 0.5,
              (region.minY() + region.maxY()) * 0.5,
              new Rotation2d()));
    }

    private void addRectRegions(RectRegion[] regions) {
      for (RectRegion region : regions) {
        addRectRegion(region);
      }
    }

    private void addHubPerimeter(Translation2d[] perimeterPoints) {
      for (int i = 0; i < perimeterPoints.length; i++) {
        Translation2d start = perimeterPoints[i];
        Translation2d end = perimeterPoints[(i + 1) % perimeterPoints.length];
        addBorderLine(start, end);
      }
    }

    private static double blueHubCenterX() {
      return GlobalConstants.FieldConstants.Hub.topCenterPoint.getX();
    }

    private static double redHubCenterX() {
      return GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.getX();
    }

    private static Translation2d[] blueHubFaceCenters() {
      return new Translation2d[] {
        inchesPoint(181.56, 134.56),
        inchesPoint(205.32, 144.32),
        inchesPoint(205.32, 158.32),
        inchesPoint(181.56, 182.08),
        inchesPoint(167.56, 182.08),
        inchesPoint(157.79, 172.32),
        inchesPoint(157.79, 158.32),
        inchesPoint(167.56, 134.56)
      };
    }

    private static Translation2d[] redHubFaceCenters() {
      return new Translation2d[] {
        inchesPoint(468.56, 134.56),
        inchesPoint(482.56, 134.56),
        inchesPoint(492.33, 144.32),
        inchesPoint(492.33, 158.32),
        inchesPoint(482.56, 182.08),
        inchesPoint(468.56, 182.08),
        inchesPoint(444.80, 172.32),
        inchesPoint(444.80, 158.32)
      };
    }

    private static Translation2d inchesPoint(double xInches, double yInches) {
      return new Translation2d(Inches.of(xInches).in(Meters), Inches.of(yInches).in(Meters));
    }
  }

  private record RectRegion(double minX, double maxX, double minY, double maxY) {}

  private static final class RebuiltFieldRegions {
    private static final double TRENCH_CENTER_X_BLUE_METERS = Inches.of(181.555).in(Meters);
    private static final double TRENCH_CENTER_X_RED_METERS = Inches.of(468.555).in(Meters);
    private static final double TRENCH_UPPER_CENTER_Y_METERS = Inches.of(291.79).in(Meters);
    private static final double TRENCH_LOWER_CENTER_Y_METERS = Inches.of(24.85).in(Meters);
    private static final double TRENCH_FOOTPRINT_DEPTH_METERS = Inches.of(24.97).in(Meters);
    private static final double TRENCH_FOOTPRINT_WIDTH_METERS = Inches.of(47.00).in(Meters);
    private static final double TRENCH_SUPPORT_DEPTH_METERS = Inches.of(12.00).in(Meters);
    private static final double TRENCH_SIDE_WALL_SPAN_METERS =
        TRENCH_FOOTPRINT_DEPTH_METERS - TRENCH_SUPPORT_DEPTH_METERS;
    private static final double TRENCH_WALL_THICKNESS_METERS = Inches.of(2.0).in(Meters);
    private static final double HINGED_TRENCH_RETURN_DEPTH_METERS = Inches.of(4.0).in(Meters);
    private static final double FIXED_TRENCH_RETURN_DEPTH_METERS = Inches.of(6.0).in(Meters);

    private RebuiltFieldRegions() {}

    private static RectRegion[] blueLeftTrenchEdgeRegions() {
      return trenchEdgeRegions(TRENCH_CENTER_X_BLUE_METERS, TRENCH_UPPER_CENTER_Y_METERS, true);
    }

    private static RectRegion[] blueRightTrenchEdgeRegions() {
      return trenchEdgeRegions(TRENCH_CENTER_X_BLUE_METERS, TRENCH_LOWER_CENTER_Y_METERS, true);
    }

    private static RectRegion[] redLeftTrenchEdgeRegions() {
      return trenchEdgeRegions(TRENCH_CENTER_X_RED_METERS, TRENCH_UPPER_CENTER_Y_METERS, false);
    }

    private static RectRegion[] redRightTrenchEdgeRegions() {
      return trenchEdgeRegions(TRENCH_CENTER_X_RED_METERS, TRENCH_LOWER_CENTER_Y_METERS, false);
    }

    private static RectRegion[] trenchEdgeRegions(
        double centerX, double centerY, boolean blueSide) {
      RectRegion open =
          centeredRect(
              centerX, centerY, TRENCH_FOOTPRINT_DEPTH_METERS, TRENCH_FOOTPRINT_WIDTH_METERS);
      RectRegion support = trenchSupportBox(centerX, centerY, blueSide);

      double sideWallMinX = blueSide ? open.minX() : open.maxX() - TRENCH_SIDE_WALL_SPAN_METERS;
      double sideWallMaxX = blueSide ? open.minX() + TRENCH_SIDE_WALL_SPAN_METERS : open.maxX();
      RectRegion upperWall =
          new RectRegion(
              sideWallMinX, sideWallMaxX, open.maxY() - TRENCH_WALL_THICKNESS_METERS, open.maxY());
      RectRegion lowerWall =
          new RectRegion(
              sideWallMinX, sideWallMaxX, open.minY(), open.minY() + TRENCH_WALL_THICKNESS_METERS);

      double returnMinX =
          blueSide ? support.minX() - HINGED_TRENCH_RETURN_DEPTH_METERS : support.maxX();
      double returnMaxX =
          blueSide ? support.minX() : support.maxX() + FIXED_TRENCH_RETURN_DEPTH_METERS;
      RectRegion upperReturn =
          new RectRegion(
              returnMinX,
              returnMaxX,
              support.maxY() - TRENCH_WALL_THICKNESS_METERS,
              support.maxY());
      RectRegion lowerReturn =
          new RectRegion(
              returnMinX,
              returnMaxX,
              support.minY(),
              support.minY() + TRENCH_WALL_THICKNESS_METERS);
      return new RectRegion[] {upperWall, lowerWall, upperReturn, lowerReturn};
    }

    private static RectRegion trenchSupportBox(double centerX, double centerY, boolean blueSide) {
      double openMinX = centerX - (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
      double openMaxX = centerX + (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
      double minX = blueSide ? openMaxX - TRENCH_SUPPORT_DEPTH_METERS : openMinX;
      double maxX = blueSide ? openMaxX : openMinX + TRENCH_SUPPORT_DEPTH_METERS;
      double minY = centerY - (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5);
      double maxY = centerY + (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5);
      return new RectRegion(minX, maxX, minY, maxY);
    }

    private static RectRegion centeredRect(
        double centerX, double centerY, double depthX, double widthY) {
      return new RectRegion(
          centerX - (depthX * 0.5),
          centerX + (depthX * 0.5),
          centerY - (widthY * 0.5),
          centerY + (widthY * 0.5));
    }
  }
}
