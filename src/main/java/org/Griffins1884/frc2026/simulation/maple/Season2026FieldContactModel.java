package org.Griffins1884.frc2026.simulation.maple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.PlanarObstacleContactModel;
import org.griffins1884.sim3d.PlanarObstacleContactSample;
import org.griffins1884.sim3d.TerrainContactModel;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainFeature;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.integration.FieldMarkerProvider;
import org.griffins1884.sim3d.integration.FieldMarkerSample;

/**
 * Standalone 2026 rebuilt field terrain/contact model.
 *
 * <p>This encodes the traversable bump surfaces and trench underpass clearance windows as explicit
 * field geometry, without depending on any robot-season repository or deploy assets.
 */
public final class Season2026FieldContactModel
    implements TerrainContactModel, PlanarObstacleContactModel, FieldMarkerProvider {
  private static final double GRADIENT_SAMPLE_METERS = 0.02;
  private static volatile boolean validationFlatTerrainOverride = false;

  private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(650.12);
  private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(316.64);

  private static final double HUB_CENTER_X_BLUE_METERS = Units.inchesToMeters(181.56);
  private static final double HUB_CENTER_X_RED_METERS = Units.inchesToMeters(468.56);
  private static final double HUB_CENTER_Y_METERS = Units.inchesToMeters(158.32);
  private static final double HUB_COLLISION_WIDTH_METERS = Units.inchesToMeters(58.41);
  private static final double HUB_COLLISION_HEIGHT_METERS = Units.inchesToMeters(47.00);
  private static final RectRegion BLUE_HUB_BOUNDS =
      centeredRect(
          HUB_CENTER_X_BLUE_METERS,
          HUB_CENTER_Y_METERS,
          HUB_COLLISION_WIDTH_METERS,
          HUB_COLLISION_HEIGHT_METERS);
  private static final RectRegion RED_HUB_BOUNDS =
      centeredRect(
          HUB_CENTER_X_RED_METERS,
          HUB_CENTER_Y_METERS,
          HUB_COLLISION_WIDTH_METERS,
          HUB_COLLISION_HEIGHT_METERS);
  private static final Translation2d[] BLUE_HUB_FACE_CENTERS =
      new Translation2d[] {
        inchesPoint(181.56, 134.56),
        inchesPoint(205.32, 144.32),
        inchesPoint(205.32, 158.32),
        inchesPoint(181.56, 182.08),
        inchesPoint(167.56, 182.08),
        inchesPoint(157.79, 172.32),
        inchesPoint(157.79, 158.32),
        inchesPoint(167.56, 134.56)
      };
  private static final Translation2d[] RED_HUB_FACE_CENTERS =
      new Translation2d[] {
        inchesPoint(468.56, 134.56),
        inchesPoint(482.56, 134.56),
        inchesPoint(492.33, 144.32),
        inchesPoint(492.33, 158.32),
        inchesPoint(482.56, 182.08),
        inchesPoint(468.56, 182.08),
        inchesPoint(444.80, 172.32),
        inchesPoint(444.80, 158.32)
      };

  private static final double BUMP_PROFILE_DEPTH_METERS = Units.inchesToMeters(48.93);
  private static final double BUMP_SPAN_METERS = Units.inchesToMeters(73.00);
  private static final double BUMP_CENTER_OFFSET_Y_METERS = Units.inchesToMeters(60.00);
  private static final RectRegion BLUE_LEFT_BUMP_BOUNDS =
      centeredRect(
          HUB_CENTER_X_BLUE_METERS,
          HUB_CENTER_Y_METERS + BUMP_CENTER_OFFSET_Y_METERS,
          BUMP_PROFILE_DEPTH_METERS,
          BUMP_SPAN_METERS);
  private static final RectRegion BLUE_RIGHT_BUMP_BOUNDS =
      centeredRect(
          HUB_CENTER_X_BLUE_METERS,
          HUB_CENTER_Y_METERS - BUMP_CENTER_OFFSET_Y_METERS,
          BUMP_PROFILE_DEPTH_METERS,
          BUMP_SPAN_METERS);
  private static final RectRegion RED_LEFT_BUMP_BOUNDS =
      centeredRect(
          HUB_CENTER_X_RED_METERS,
          HUB_CENTER_Y_METERS + BUMP_CENTER_OFFSET_Y_METERS,
          BUMP_PROFILE_DEPTH_METERS,
          BUMP_SPAN_METERS);
  private static final RectRegion RED_RIGHT_BUMP_BOUNDS =
      centeredRect(
          HUB_CENTER_X_RED_METERS,
          HUB_CENTER_Y_METERS - BUMP_CENTER_OFFSET_Y_METERS,
          BUMP_PROFILE_DEPTH_METERS,
          BUMP_SPAN_METERS);

  private static final double TRENCH_CENTER_X_BLUE_METERS = Units.inchesToMeters(181.555);
  private static final double TRENCH_CENTER_X_RED_METERS = Units.inchesToMeters(468.555);
  private static final double TRENCH_UPPER_CENTER_Y_METERS = Units.inchesToMeters(291.79);
  private static final double TRENCH_LOWER_CENTER_Y_METERS = Units.inchesToMeters(24.85);
  private static final double TRENCH_FOOTPRINT_DEPTH_METERS = Units.inchesToMeters(24.97);
  private static final double TRENCH_FOOTPRINT_WIDTH_METERS = Units.inchesToMeters(47.00);
  private static final double TRENCH_SUPPORT_DEPTH_METERS = Units.inchesToMeters(12.00);
  private static final double TRENCH_SIDE_WALL_SPAN_METERS =
      TRENCH_FOOTPRINT_DEPTH_METERS - TRENCH_SUPPORT_DEPTH_METERS;
  private static final double TRENCH_WALL_THICKNESS_METERS = Units.inchesToMeters(2.0);
  private static final double HINGED_TRENCH_RETURN_DEPTH_METERS = Units.inchesToMeters(4.0);
  private static final double FIXED_TRENCH_RETURN_DEPTH_METERS = Units.inchesToMeters(6.0);

  private static final RectRegion BLUE_LEFT_TRENCH_OPEN_BOUNDS =
      centeredRect(
          TRENCH_CENTER_X_BLUE_METERS,
          TRENCH_UPPER_CENTER_Y_METERS,
          TRENCH_FOOTPRINT_DEPTH_METERS,
          TRENCH_FOOTPRINT_WIDTH_METERS);
  private static final RectRegion BLUE_RIGHT_TRENCH_OPEN_BOUNDS =
      centeredRect(
          TRENCH_CENTER_X_BLUE_METERS,
          TRENCH_LOWER_CENTER_Y_METERS,
          TRENCH_FOOTPRINT_DEPTH_METERS,
          TRENCH_FOOTPRINT_WIDTH_METERS);
  private static final RectRegion RED_LEFT_TRENCH_OPEN_BOUNDS =
      centeredRect(
          TRENCH_CENTER_X_RED_METERS,
          TRENCH_UPPER_CENTER_Y_METERS,
          TRENCH_FOOTPRINT_DEPTH_METERS,
          TRENCH_FOOTPRINT_WIDTH_METERS);
  private static final RectRegion RED_RIGHT_TRENCH_OPEN_BOUNDS =
      centeredRect(
          TRENCH_CENTER_X_RED_METERS,
          TRENCH_LOWER_CENTER_Y_METERS,
          TRENCH_FOOTPRINT_DEPTH_METERS,
          TRENCH_FOOTPRINT_WIDTH_METERS);

  private static final RectRegion BLUE_LEFT_TRENCH_BLOCK_BOUNDS =
      trenchSupportBox(TRENCH_CENTER_X_BLUE_METERS, TRENCH_UPPER_CENTER_Y_METERS, true);
  private static final RectRegion BLUE_RIGHT_TRENCH_BLOCK_BOUNDS =
      trenchSupportBox(TRENCH_CENTER_X_BLUE_METERS, TRENCH_LOWER_CENTER_Y_METERS, true);
  private static final RectRegion RED_LEFT_TRENCH_BLOCK_BOUNDS =
      trenchSupportBox(TRENCH_CENTER_X_RED_METERS, TRENCH_UPPER_CENTER_Y_METERS, false);
  private static final RectRegion RED_RIGHT_TRENCH_BLOCK_BOUNDS =
      trenchSupportBox(TRENCH_CENTER_X_RED_METERS, TRENCH_LOWER_CENTER_Y_METERS, false);

  private static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(6.513);
  private static final double BUMP_RAMP_FRACTION = 24.47 / 48.93;
  private static final double TRENCH_OPENING_HEIGHT_METERS = Units.inchesToMeters(22.25);
  private static final double TOWER_INNER_OPENING_WIDTH_METERS = Units.inchesToMeters(32.250);
  private static final double TOWER_UPRIGHT_OFFSET_METERS = Units.inchesToMeters(0.75);
  private static final double TOWER_UPRIGHT_HEIGHT_METERS = Units.inchesToMeters(72.1);

  private static final double TOWER_FRONT_FACE_X_METERS = Units.inchesToMeters(0.55);
  private static final double RED_TOWER_FRONT_FACE_X_METERS = Units.inchesToMeters(649.57);
  private static final double TOWER_WIDTH_METERS = Units.inchesToMeters(47.00);
  private static final double TOWER_DEPTH_METERS = Units.inchesToMeters(40.00);
  private static final double TOWER_SIDE_WIDTH_METERS =
      (TOWER_WIDTH_METERS - TOWER_INNER_OPENING_WIDTH_METERS) * 0.5;
  private static final double BLUE_TOWER_CENTER_Y_METERS =
      Units.inchesToMeters((146.86 + 163.86) * 0.5);
  private static final double RED_TOWER_CENTER_Y_METERS =
      Units.inchesToMeters((152.78 + 169.78) * 0.5);

  public static final Season2026FieldContactModel INSTANCE = new Season2026FieldContactModel();

  private Season2026FieldContactModel() {}

  public static void setValidationFlatTerrainOverride(boolean enabled) {
    validationFlatTerrainOverride = enabled;
  }

  @Override
  public TerrainSample sample(Pose2d robotPose) {
    if (validationFlatTerrainOverride) {
      return new TerrainSample(
          new Pose3d(
              robotPose.getX(),
              robotPose.getY(),
              0.0,
              new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())),
          0.0,
          0.0,
          0.0);
    }
    Translation2d translation = robotPose.getTranslation();
    double z = bumpHeightMeters(translation);
    double[] gradient = terrainGradient(translation);

    double yaw = robotPose.getRotation().getRadians();
    double forwardSlope = (gradient[0] * Math.cos(yaw)) + (gradient[1] * Math.sin(yaw));
    double rightSlope = (-gradient[0] * Math.sin(yaw)) + (gradient[1] * Math.cos(yaw));

    double pitch = -Math.atan(forwardSlope);
    double roll = -Math.atan(rightSlope);
    Pose3d pose3d =
        new Pose3d(robotPose.getX(), robotPose.getY(), z, new Rotation3d(roll, pitch, yaw));
    return new TerrainSample(pose3d, roll, pitch, z);
  }

  @Override
  public TerrainContactSample sampleContact(Pose2d robotPose, ChassisFootprint chassisFootprint) {
    if (validationFlatTerrainOverride) {
      TerrainSample terrainSample = sample(robotPose);
      return new TerrainContactSample(
          terrainSample,
          TerrainFeature.FLAT,
          Double.POSITIVE_INFINITY,
          chassisFootprint.groundClearanceMeters(),
          Double.POSITIVE_INFINITY,
          true,
          true);
    }
    TerrainSample terrainSample = sample(robotPose);
    TerrainFeature feature = featureAt(robotPose.getTranslation());
    double overheadClearanceMeters = overheadClearanceMeters(feature);
    double underbodyClearanceMarginMeters =
        chassisFootprint.groundClearanceMeters() - terrainSample.heightMeters();
    double overheadClearanceMarginMeters =
        Double.isFinite(overheadClearanceMeters)
            ? overheadClearanceMeters - chassisFootprint.heightMeters()
            : Double.POSITIVE_INFINITY;
    boolean traversableSurface = traversableSurface(feature);
    boolean clearanceSatisfied = traversableSurface && overheadClearanceMarginMeters >= 0.0;
    return new TerrainContactSample(
        terrainSample,
        feature,
        overheadClearanceMeters,
        underbodyClearanceMarginMeters,
        overheadClearanceMarginMeters,
        traversableSurface,
        clearanceSatisfied);
  }

  @Override
  public FieldMarkerSample[] getFieldMarkers() {
    List<FieldMarkerSample> markers = new ArrayList<>();
    addHubMarkers(markers);
    addBumpMarkers(markers);
    addTrenchMarkers(markers);
    addTrenchEdgeMarkers(markers);
    addTowerMarkers(markers);
    return markers.toArray(FieldMarkerSample[]::new);
  }

  @Override
  public PlanarObstacleContactSample[] samplePlanarObstacleContacts(
      Translation2d point, double contactHeightMeters) {
    if (contactHeightMeters < 0.0) {
      return new PlanarObstacleContactSample[0];
    }

    List<PlanarObstacleContactSample> contacts = new ArrayList<>();
    addPolygonContact(
        contacts,
        TerrainFeature.BLUE_HUB,
        BLUE_HUB_FACE_CENTERS,
        HUB_COLLISION_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addPolygonContact(
        contacts,
        TerrainFeature.RED_HUB,
        RED_HUB_FACE_CENTERS,
        HUB_COLLISION_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.BLUE_LEFT_TRENCH_EDGE,
        blueLeftTrenchEdgeRegions(),
        TRENCH_OPENING_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.BLUE_RIGHT_TRENCH_EDGE,
        blueRightTrenchEdgeRegions(),
        TRENCH_OPENING_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.RED_LEFT_TRENCH_EDGE,
        redLeftTrenchEdgeRegions(),
        TRENCH_OPENING_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.RED_RIGHT_TRENCH_EDGE,
        redRightTrenchEdgeRegions(),
        TRENCH_OPENING_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.BLUE_TOWER,
        new RectRegion[] {blueTowerLeftWall(), blueTowerRightWall(), blueTowerBackWall()},
        TOWER_UPRIGHT_HEIGHT_METERS,
        point,
        contactHeightMeters);
    addRectContacts(
        contacts,
        TerrainFeature.RED_TOWER,
        new RectRegion[] {redTowerLeftWall(), redTowerRightWall(), redTowerBackWall()},
        TOWER_UPRIGHT_HEIGHT_METERS,
        point,
        contactHeightMeters);
    return contacts.toArray(PlanarObstacleContactSample[]::new);
  }

  public TerrainFeature featureAt(Translation2d position) {
    if (inHub(BLUE_HUB_BOUNDS, BLUE_HUB_FACE_CENTERS, position)) {
      return TerrainFeature.BLUE_HUB;
    }
    if (inHub(RED_HUB_BOUNDS, RED_HUB_FACE_CENTERS, position)) {
      return TerrainFeature.RED_HUB;
    }
    if (inBlueTower(position)) {
      return TerrainFeature.BLUE_TOWER;
    }
    if (inRedTower(position)) {
      return TerrainFeature.RED_TOWER;
    }
    if (inTrenchEdge(blueLeftTrenchEdgeRegions(), position)) {
      return TerrainFeature.BLUE_LEFT_TRENCH_EDGE;
    }
    if (inTrenchEdge(blueRightTrenchEdgeRegions(), position)) {
      return TerrainFeature.BLUE_RIGHT_TRENCH_EDGE;
    }
    if (inTrenchEdge(redLeftTrenchEdgeRegions(), position)) {
      return TerrainFeature.RED_LEFT_TRENCH_EDGE;
    }
    if (inTrenchEdge(redRightTrenchEdgeRegions(), position)) {
      return TerrainFeature.RED_RIGHT_TRENCH_EDGE;
    }
    if (contains(BLUE_LEFT_BUMP_BOUNDS, position)) {
      return TerrainFeature.BLUE_LEFT_BUMP;
    }
    if (contains(BLUE_RIGHT_BUMP_BOUNDS, position)) {
      return TerrainFeature.BLUE_RIGHT_BUMP;
    }
    if (contains(RED_LEFT_BUMP_BOUNDS, position)) {
      return TerrainFeature.RED_LEFT_BUMP;
    }
    if (contains(RED_RIGHT_BUMP_BOUNDS, position)) {
      return TerrainFeature.RED_RIGHT_BUMP;
    }
    if (contains(BLUE_LEFT_TRENCH_OPEN_BOUNDS, position)) {
      return TerrainFeature.BLUE_LEFT_TRENCH;
    }
    if (contains(BLUE_RIGHT_TRENCH_OPEN_BOUNDS, position)) {
      return TerrainFeature.BLUE_RIGHT_TRENCH;
    }
    if (contains(RED_LEFT_TRENCH_OPEN_BOUNDS, position)) {
      return TerrainFeature.RED_LEFT_TRENCH;
    }
    if (contains(RED_RIGHT_TRENCH_OPEN_BOUNDS, position)) {
      return TerrainFeature.RED_RIGHT_TRENCH;
    }
    return TerrainFeature.FLAT;
  }

  public double bumpHeightMeters(Translation2d fieldTranslation) {
    return Math.max(
        Math.max(
            profileHeight(fieldTranslation, BLUE_LEFT_BUMP_BOUNDS),
            profileHeight(fieldTranslation, BLUE_RIGHT_BUMP_BOUNDS)),
        Math.max(
            profileHeight(fieldTranslation, RED_LEFT_BUMP_BOUNDS),
            profileHeight(fieldTranslation, RED_RIGHT_BUMP_BOUNDS)));
  }

  private double[] terrainGradient(Translation2d point) {
    double dzdx =
        (bumpHeightMeters(point.plus(new Translation2d(GRADIENT_SAMPLE_METERS, 0.0)))
                - bumpHeightMeters(point.minus(new Translation2d(GRADIENT_SAMPLE_METERS, 0.0))))
            / (2.0 * GRADIENT_SAMPLE_METERS);
    double dzdy =
        (bumpHeightMeters(point.plus(new Translation2d(0.0, GRADIENT_SAMPLE_METERS)))
                - bumpHeightMeters(point.minus(new Translation2d(0.0, GRADIENT_SAMPLE_METERS))))
            / (2.0 * GRADIENT_SAMPLE_METERS);
    return new double[] {dzdx, dzdy};
  }

  private double profileHeight(Translation2d point, RectRegion bounds) {
    if (!contains(bounds, point)) {
      return 0.0;
    }
    double progress = normalizedProgress(bounds.minX, bounds.maxX, point.getX());
    double rampFraction = Math.min(BUMP_RAMP_FRACTION, 0.5);
    if (progress <= rampFraction) {
      return BUMP_HEIGHT_METERS * (progress / rampFraction);
    }
    if (progress >= 1.0 - rampFraction) {
      return BUMP_HEIGHT_METERS * ((1.0 - progress) / rampFraction);
    }
    return BUMP_HEIGHT_METERS;
  }

  private double overheadClearanceMeters(TerrainFeature feature) {
    return switch (feature) {
      case BLUE_LEFT_TRENCH, BLUE_RIGHT_TRENCH, RED_LEFT_TRENCH, RED_RIGHT_TRENCH ->
          TRENCH_OPENING_HEIGHT_METERS;
      default -> Double.POSITIVE_INFINITY;
    };
  }

  private boolean traversableSurface(TerrainFeature feature) {
    return switch (feature) {
      case BLUE_HUB,
              RED_HUB,
              BLUE_TOWER,
              RED_TOWER,
              BLUE_LEFT_TRENCH_EDGE,
              BLUE_RIGHT_TRENCH_EDGE,
              RED_LEFT_TRENCH_EDGE,
              RED_RIGHT_TRENCH_EDGE ->
          false;
      default -> true;
    };
  }

  private static boolean contains(RectRegion region, Translation2d point) {
    return point.getX() >= region.minX
        && point.getX() <= region.maxX
        && point.getY() >= region.minY
        && point.getY() <= region.maxY;
  }

  private static boolean inHub(
      RectRegion hubBounds, Translation2d[] hubFaceCenters, Translation2d point) {
    return contains(hubBounds, point) && pointInPolygon(hubFaceCenters, point);
  }

  private static boolean inTrenchEdge(RectRegion[] regions, Translation2d point) {
    for (RectRegion region : regions) {
      if (contains(region, point)) {
        return true;
      }
    }
    return false;
  }

  private static double normalizedProgress(double min, double max, double value) {
    double span = Math.max(max - min, 1e-9);
    return Math.max(0.0, Math.min(1.0, (value - min) / span));
  }

  private static boolean pointInPolygon(Translation2d[] polygon, Translation2d point) {
    boolean inside = false;
    for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      Translation2d current = polygon[i];
      Translation2d previous = polygon[j];
      boolean crossesScanline = (current.getY() > point.getY()) != (previous.getY() > point.getY());
      if (!crossesScanline) {
        continue;
      }

      double edgeCrossX =
          ((previous.getX() - current.getX()) * (point.getY() - current.getY()))
                  / (previous.getY() - current.getY())
              + current.getX();
      if (point.getX() < edgeCrossX) {
        inside = !inside;
      }
    }
    return inside;
  }

  private static void addRectContacts(
      List<PlanarObstacleContactSample> contacts,
      TerrainFeature feature,
      RectRegion[] regions,
      double obstacleHeightMeters,
      Translation2d point,
      double contactHeightMeters) {
    if (contactHeightMeters > obstacleHeightMeters) {
      return;
    }
    for (RectRegion region : regions) {
      addRectContactSamples(contacts, feature, region, obstacleHeightMeters, point);
    }
  }

  private static void addRectContactSamples(
      List<PlanarObstacleContactSample> contacts,
      TerrainFeature feature,
      RectRegion region,
      double obstacleHeightMeters,
      Translation2d point) {
    if (!contains(region, point)) {
      return;
    }

    double leftPenetration = point.getX() - region.minX;
    double rightPenetration = region.maxX - point.getX();
    double bottomPenetration = point.getY() - region.minY;
    double topPenetration = region.maxY - point.getY();
    contacts.add(
        new PlanarObstacleContactSample(
            feature,
            new Translation2d(-1.0, 0.0),
            Math.max(0.0, leftPenetration),
            obstacleHeightMeters));
    contacts.add(
        new PlanarObstacleContactSample(
            feature,
            new Translation2d(1.0, 0.0),
            Math.max(0.0, rightPenetration),
            obstacleHeightMeters));
    contacts.add(
        new PlanarObstacleContactSample(
            feature,
            new Translation2d(0.0, -1.0),
            Math.max(0.0, bottomPenetration),
            obstacleHeightMeters));
    contacts.add(
        new PlanarObstacleContactSample(
            feature,
            new Translation2d(0.0, 1.0),
            Math.max(0.0, topPenetration),
            obstacleHeightMeters));
  }

  private static void addPolygonContact(
      List<PlanarObstacleContactSample> contacts,
      TerrainFeature feature,
      Translation2d[] polygon,
      double obstacleHeightMeters,
      Translation2d point,
      double contactHeightMeters) {
    if (contactHeightMeters > obstacleHeightMeters || !pointInPolygon(polygon, point)) {
      return;
    }

    boolean counterClockwise = signedPolygonArea(polygon) >= 0.0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d start = polygon[i];
      Translation2d end = polygon[(i + 1) % polygon.length];
      Translation2d edge = end.minus(start);
      double edgeLength = edge.getNorm();
      if (edgeLength <= 1e-9) {
        continue;
      }

      Translation2d outwardNormal =
          counterClockwise
              ? new Translation2d(edge.getY() / edgeLength, -edge.getX() / edgeLength)
              : new Translation2d(-edge.getY() / edgeLength, edge.getX() / edgeLength);
      double penetration = -outwardNormal.dot(point.minus(start));
      contacts.add(
          new PlanarObstacleContactSample(
              feature, outwardNormal, Math.max(0.0, penetration), obstacleHeightMeters));
    }
  }

  private static double signedPolygonArea(Translation2d[] polygon) {
    double areaTwice = 0.0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d current = polygon[i];
      Translation2d next = polygon[(i + 1) % polygon.length];
      areaTwice += (current.getX() * next.getY()) - (next.getX() * current.getY());
    }
    return areaTwice * 0.5;
  }

  private static double hubCenterXBlue() {
    return HUB_CENTER_X_BLUE_METERS;
  }

  private static double hubCenterXRed() {
    return HUB_CENTER_X_RED_METERS;
  }

  private static boolean inBlueTower(Translation2d position) {
    return contains(blueTowerLeftWall(), position)
        || contains(blueTowerRightWall(), position)
        || contains(blueTowerBackWall(), position);
  }

  private static boolean inRedTower(Translation2d position) {
    return contains(redTowerLeftWall(), position)
        || contains(redTowerRightWall(), position)
        || contains(redTowerBackWall(), position);
  }

  private static RectRegion blueTowerLeftWall() {
    double towerCenterX = TOWER_FRONT_FACE_X_METERS - (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        BLUE_TOWER_CENTER_Y_METERS
            + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            + (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion blueTowerRightWall() {
    double towerCenterX = TOWER_FRONT_FACE_X_METERS - (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        BLUE_TOWER_CENTER_Y_METERS
            - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            - (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion blueTowerBackWall() {
    return centeredRect(
        TOWER_FRONT_FACE_X_METERS - TOWER_DEPTH_METERS,
        BLUE_TOWER_CENTER_Y_METERS,
        Units.inchesToMeters(2.0),
        TOWER_WIDTH_METERS);
  }

  private static RectRegion redTowerLeftWall() {
    double towerCenterX = RED_TOWER_FRONT_FACE_X_METERS + (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        RED_TOWER_CENTER_Y_METERS
            + (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            + (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion redTowerRightWall() {
    double towerCenterX = RED_TOWER_FRONT_FACE_X_METERS + (TOWER_DEPTH_METERS * 0.5);
    double centerY =
        RED_TOWER_CENTER_Y_METERS
            - (TOWER_INNER_OPENING_WIDTH_METERS * 0.5)
            - (TOWER_SIDE_WIDTH_METERS * 0.5);
    return centeredRect(towerCenterX, centerY, TOWER_DEPTH_METERS, TOWER_SIDE_WIDTH_METERS);
  }

  private static RectRegion redTowerBackWall() {
    return centeredRect(
        RED_TOWER_FRONT_FACE_X_METERS + TOWER_DEPTH_METERS,
        RED_TOWER_CENTER_Y_METERS,
        Units.inchesToMeters(2.0),
        TOWER_WIDTH_METERS);
  }

  private static RectRegion centeredRect(
      double centerX, double centerY, double widthMeters, double heightMeters) {
    return new RectRegion(
        centerX - (widthMeters * 0.5),
        centerX + (widthMeters * 0.5),
        centerY - (heightMeters * 0.5),
        centerY + (heightMeters * 0.5));
  }

  private static RectRegion trenchSupportBox(
      double trenchCenterX, double trenchCenterY, boolean blueSide) {
    double trenchMinX = trenchCenterX - (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double trenchMaxX = trenchCenterX + (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double supportCenterX =
        blueSide
            ? trenchMaxX - (TRENCH_SUPPORT_DEPTH_METERS * 0.5)
            : trenchMinX + (TRENCH_SUPPORT_DEPTH_METERS * 0.5);
    return centeredRect(
        supportCenterX, trenchCenterY, TRENCH_SUPPORT_DEPTH_METERS, TRENCH_FOOTPRINT_WIDTH_METERS);
  }

  private static Segment[] trenchWallSegments(
      double trenchCenterX, double trenchCenterY, boolean blueSide) {
    double trenchMinX = trenchCenterX - (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double trenchMaxX = trenchCenterX + (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double supportBoundaryX =
        blueSide
            ? trenchMaxX - TRENCH_SUPPORT_DEPTH_METERS
            : trenchMinX + TRENCH_SUPPORT_DEPTH_METERS;
    double openMinX = blueSide ? trenchMinX : supportBoundaryX;
    double openMaxX = blueSide ? supportBoundaryX : trenchMaxX;
    double topY = trenchCenterY + (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5);
    double bottomY = trenchCenterY - (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5);

    return new Segment[] {
      new Segment(new Translation2d(openMinX, topY), new Translation2d(openMaxX, topY)),
      new Segment(new Translation2d(openMinX, bottomY), new Translation2d(openMaxX, bottomY))
    };
  }

  private static RectRegion[] trenchEdgeRegions(
      double trenchCenterX, double trenchCenterY, boolean blueSide, boolean hingedAssembly) {
    double trenchMinX = trenchCenterX - (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double trenchMaxX = trenchCenterX + (TRENCH_FOOTPRINT_DEPTH_METERS * 0.5);
    double supportBoundaryX =
        blueSide
            ? trenchMaxX - TRENCH_SUPPORT_DEPTH_METERS
            : trenchMinX + TRENCH_SUPPORT_DEPTH_METERS;
    double openMinX = blueSide ? trenchMinX : supportBoundaryX;
    double openMaxX = blueSide ? supportBoundaryX : trenchMaxX;
    double topWallCenterY =
        trenchCenterY
            + (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5)
            - (TRENCH_WALL_THICKNESS_METERS * 0.5);
    double bottomWallCenterY =
        trenchCenterY
            - (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5)
            + (TRENCH_WALL_THICKNESS_METERS * 0.5);
    double returnDepthMeters =
        hingedAssembly ? HINGED_TRENCH_RETURN_DEPTH_METERS : FIXED_TRENCH_RETURN_DEPTH_METERS;
    double returnCenterX =
        blueSide ? openMinX + (returnDepthMeters * 0.5) : openMaxX - (returnDepthMeters * 0.5);

    return new RectRegion[] {
      trenchSupportBox(trenchCenterX, trenchCenterY, blueSide),
      centeredRect(
          (openMinX + openMaxX) * 0.5,
          topWallCenterY,
          openMaxX - openMinX,
          TRENCH_WALL_THICKNESS_METERS),
      centeredRect(
          (openMinX + openMaxX) * 0.5,
          bottomWallCenterY,
          openMaxX - openMinX,
          TRENCH_WALL_THICKNESS_METERS),
      centeredRect(
          returnCenterX,
          trenchCenterY + (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5) - (returnDepthMeters * 0.5),
          TRENCH_WALL_THICKNESS_METERS,
          returnDepthMeters),
      centeredRect(
          returnCenterX,
          trenchCenterY - (TRENCH_FOOTPRINT_WIDTH_METERS * 0.5) + (returnDepthMeters * 0.5),
          TRENCH_WALL_THICKNESS_METERS,
          returnDepthMeters)
    };
  }

  public static double fieldLengthMeters() {
    return FIELD_LENGTH_METERS;
  }

  public static double fieldWidthMeters() {
    return FIELD_WIDTH_METERS;
  }

  public static double hubCenterXBlueMeters() {
    return hubCenterXBlue();
  }

  public static double hubCenterXRedMeters() {
    return hubCenterXRed();
  }

  public static double hubCenterYMeters() {
    return HUB_CENTER_Y_METERS;
  }

  public static double hubCollisionSizeMeters() {
    return (BLUE_HUB_BOUNDS.maxX
            - BLUE_HUB_BOUNDS.minX
            + BLUE_HUB_BOUNDS.maxY
            - BLUE_HUB_BOUNDS.minY)
        * 0.25;
  }

  public static double hubCollisionWidthMeters() {
    return HUB_COLLISION_WIDTH_METERS;
  }

  public static double hubCollisionHeightMeters() {
    return HUB_COLLISION_HEIGHT_METERS;
  }

  public static double blueTowerFrontFaceXMeters() {
    return TOWER_FRONT_FACE_X_METERS;
  }

  public static double redTowerFrontFaceXMeters() {
    return RED_TOWER_FRONT_FACE_X_METERS;
  }

  public static double towerWidthMeters() {
    return TOWER_WIDTH_METERS;
  }

  public static double towerDepthMeters() {
    return TOWER_DEPTH_METERS;
  }

  public static double blueTowerCenterYMeters() {
    return BLUE_TOWER_CENTER_Y_METERS;
  }

  public static double redTowerCenterYMeters() {
    return RED_TOWER_CENTER_Y_METERS;
  }

  public static double blueTrenchCenterXMeters() {
    return TRENCH_CENTER_X_BLUE_METERS;
  }

  public static double redTrenchCenterXMeters() {
    return TRENCH_CENTER_X_RED_METERS;
  }

  public static double trenchDepthMeters() {
    return TRENCH_FOOTPRINT_DEPTH_METERS;
  }

  public static double blueLeftTrenchBlockCenterYMeters() {
    return BLUE_LEFT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double blueRightTrenchBlockCenterYMeters() {
    return BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double redLeftTrenchBlockCenterYMeters() {
    return RED_LEFT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double redRightTrenchBlockCenterYMeters() {
    return RED_RIGHT_TRENCH_BLOCK_BOUNDS.centerY();
  }

  public static double leftTrenchBlockWidthMeters() {
    return BLUE_LEFT_TRENCH_BLOCK_BOUNDS.maxY - BLUE_LEFT_TRENCH_BLOCK_BOUNDS.minY;
  }

  public static double rightTrenchBlockWidthMeters() {
    return BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.maxY - BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.minY;
  }

  public static double blueLeftTrenchCenterYMeters() {
    return TRENCH_UPPER_CENTER_Y_METERS;
  }

  public static double blueRightTrenchCenterYMeters() {
    return TRENCH_LOWER_CENTER_Y_METERS;
  }

  public static double redLeftTrenchCenterYMeters() {
    return TRENCH_UPPER_CENTER_Y_METERS;
  }

  public static double redRightTrenchCenterYMeters() {
    return TRENCH_LOWER_CENTER_Y_METERS;
  }

  public static RectRegion blueLeftTrenchSupportBox() {
    return BLUE_LEFT_TRENCH_BLOCK_BOUNDS;
  }

  public static RectRegion blueRightTrenchSupportBox() {
    return BLUE_RIGHT_TRENCH_BLOCK_BOUNDS;
  }

  public static RectRegion redLeftTrenchSupportBox() {
    return RED_LEFT_TRENCH_BLOCK_BOUNDS;
  }

  public static RectRegion redRightTrenchSupportBox() {
    return RED_RIGHT_TRENCH_BLOCK_BOUNDS;
  }

  public static Segment[] blueLeftTrenchWalls() {
    return trenchWallSegments(TRENCH_CENTER_X_BLUE_METERS, TRENCH_UPPER_CENTER_Y_METERS, true);
  }

  public static Segment[] blueRightTrenchWalls() {
    return trenchWallSegments(TRENCH_CENTER_X_BLUE_METERS, TRENCH_LOWER_CENTER_Y_METERS, true);
  }

  public static Segment[] redLeftTrenchWalls() {
    return trenchWallSegments(TRENCH_CENTER_X_RED_METERS, TRENCH_UPPER_CENTER_Y_METERS, false);
  }

  public static Segment[] redRightTrenchWalls() {
    return trenchWallSegments(TRENCH_CENTER_X_RED_METERS, TRENCH_LOWER_CENTER_Y_METERS, false);
  }

  public static RectRegion[] blueLeftTrenchEdgeRegions() {
    return trenchEdgeRegions(TRENCH_CENTER_X_BLUE_METERS, TRENCH_UPPER_CENTER_Y_METERS, true, true);
  }

  public static RectRegion[] blueRightTrenchEdgeRegions() {
    return trenchEdgeRegions(
        TRENCH_CENTER_X_BLUE_METERS, TRENCH_LOWER_CENTER_Y_METERS, true, false);
  }

  public static RectRegion[] redLeftTrenchEdgeRegions() {
    return trenchEdgeRegions(TRENCH_CENTER_X_RED_METERS, TRENCH_UPPER_CENTER_Y_METERS, false, true);
  }

  public static RectRegion[] redRightTrenchEdgeRegions() {
    return trenchEdgeRegions(
        TRENCH_CENTER_X_RED_METERS, TRENCH_LOWER_CENTER_Y_METERS, false, false);
  }

  public static Translation2d[] blueHubFaceCenters() {
    return BLUE_HUB_FACE_CENTERS.clone();
  }

  public static Translation2d[] redHubFaceCenters() {
    return RED_HUB_FACE_CENTERS.clone();
  }

  private static void addHubMarkers(List<FieldMarkerSample> markers) {
    addMarker(markers, "blue-hub", BLUE_HUB_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-hub", RED_HUB_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
  }

  private static void addBumpMarkers(List<FieldMarkerSample> markers) {
    addMarker(markers, "blue-left-bump", BLUE_LEFT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(
        markers, "blue-right-bump", BLUE_RIGHT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-left-bump", RED_LEFT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
    addMarker(markers, "red-right-bump", RED_RIGHT_BUMP_BOUNDS.center3d(BUMP_HEIGHT_METERS * 0.5));
  }

  private static void addTrenchMarkers(List<FieldMarkerSample> markers) {
    addMarker(
        markers,
        "blue-left-trench-open",
        BLUE_LEFT_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "blue-right-trench-open",
        BLUE_RIGHT_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-left-trench-open",
        RED_LEFT_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
    addMarker(
        markers,
        "red-right-trench-open",
        RED_RIGHT_TRENCH_OPEN_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS));
  }

  private static void addTrenchEdgeMarkers(List<FieldMarkerSample> markers) {
    addMarker(
        markers,
        "blue-left-trench-edge",
        BLUE_LEFT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "blue-right-trench-edge",
        BLUE_RIGHT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-left-trench-edge",
        RED_LEFT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-right-trench-edge",
        RED_RIGHT_TRENCH_BLOCK_BOUNDS.center3d(TRENCH_OPENING_HEIGHT_METERS * 0.5));
  }

  private static void addTowerMarkers(List<FieldMarkerSample> markers) {
    double blueLeftUprightY =
        BLUE_TOWER_CENTER_Y_METERS
            + TOWER_INNER_OPENING_WIDTH_METERS * 0.5
            + TOWER_UPRIGHT_OFFSET_METERS;
    double blueRightUprightY =
        BLUE_TOWER_CENTER_Y_METERS
            - TOWER_INNER_OPENING_WIDTH_METERS * 0.5
            - TOWER_UPRIGHT_OFFSET_METERS;
    double redLeftUprightY =
        RED_TOWER_CENTER_Y_METERS
            + TOWER_INNER_OPENING_WIDTH_METERS * 0.5
            + TOWER_UPRIGHT_OFFSET_METERS;
    double redRightUprightY =
        RED_TOWER_CENTER_Y_METERS
            - TOWER_INNER_OPENING_WIDTH_METERS * 0.5
            - TOWER_UPRIGHT_OFFSET_METERS;

    addMarker(
        markers,
        "blue-tower-left-upright",
        new Translation3d(
            TOWER_FRONT_FACE_X_METERS, blueLeftUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "blue-tower-right-upright",
        new Translation3d(
            TOWER_FRONT_FACE_X_METERS, blueRightUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-tower-left-upright",
        new Translation3d(
            RED_TOWER_FRONT_FACE_X_METERS, redLeftUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-tower-right-upright",
        new Translation3d(
            RED_TOWER_FRONT_FACE_X_METERS, redRightUprightY, TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "blue-tower-back-wall",
        blueTowerBackWall().center3d(TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
    addMarker(
        markers,
        "red-tower-back-wall",
        redTowerBackWall().center3d(TOWER_UPRIGHT_HEIGHT_METERS * 0.5));
  }

  private static void addMarker(
      List<FieldMarkerSample> markers, String id, Translation3d translation3d) {
    markers.add(new FieldMarkerSample(id, new Pose3d(translation3d, new Rotation3d())));
  }

  private static Translation2d inchesPoint(double xInches, double yInches) {
    return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
  }

  public record RectRegion(double minX, double maxX, double minY, double maxY) {
    Translation3d center3d(double z) {
      return new Translation3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, z);
    }

    double centerX() {
      return (minX + maxX) * 0.5;
    }

    double centerY() {
      return (minY + maxY) * 0.5;
    }
  }

  public record Segment(Translation2d start, Translation2d end) {}
}
