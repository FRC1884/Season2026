package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.simulation.engine.CollisionSystem;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.Griffins1884.frc2026.simulation.sensors.LatencyQueue;
import org.Griffins1884.frc2026.simulation.sensors.SeededGaussianNoise;

/** Deterministic AprilTag camera simulation driven from the authoritative local physics world. */
public final class AprilTagVisionIOSim implements VisionIO {
  private static final long LATENCY_NANOS = 60_000_000L;
  private static final long FRAME_PERIOD_NANOS = 33_333_333L;
  private static final double HORIZONTAL_FOV_RADIANS = Math.toRadians(100.0);
  private static final double VERTICAL_FOV_RADIANS = Math.toRadians(70.0);
  private static final double BASE_XY_NOISE_METERS = 1.0;
  private static final double BASE_THETA_NOISE_RADIANS = 1.0;

  private final CameraConstants cameraConstants;
  private final LocalSwervePhysicsSimulation simulation;
  private final CollisionSystem collisionSystem = new CollisionSystem();
  private final LatencyQueue<Packet> latencyQueue = new LatencyQueue<>();
  private final SeededGaussianNoise xNoise;
  private final SeededGaussianNoise yNoise;
  private final SeededGaussianNoise thetaNoise;
  private long lastCaptureNanos = Long.MIN_VALUE;
  private Packet current = Packet.empty();

  public AprilTagVisionIOSim(
      CameraConstants cameraConstants, LocalSwervePhysicsSimulation simulation) {
    this.cameraConstants = cameraConstants;
    this.simulation = simulation;
    long seed = 0x2026_1884L ^ cameraConstants.cameraName().hashCode();
    xNoise = new SeededGaussianNoise(seed ^ 0x51L, BASE_XY_NOISE_METERS);
    yNoise = new SeededGaussianNoise(seed ^ 0x73L, BASE_XY_NOISE_METERS);
    thetaNoise = new SeededGaussianNoise(seed ^ 0x95L, BASE_THETA_NOISE_RADIANS);
  }

  @Override
  public CameraConstants getCameraConstants() {
    return cameraConstants;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long nowNanos = Math.round(simulation.getSimTimeSeconds() * 1_000_000_000.0);
    if (nowNanos - lastCaptureNanos >= FRAME_PERIOD_NANOS) {
      latencyQueue.enqueue(nowNanos + LATENCY_NANOS, capturePacket(nowNanos));
      lastCaptureNanos = nowNanos;
    }
    List<Packet> ready = latencyQueue.releaseReady(nowNanos);
    if (!ready.isEmpty()) {
      current = ready.get(ready.size() - 1);
    }

    inputs.connected = true;
    inputs.seesTarget = current.seesTarget;
    inputs.megatagCount = current.megatagCount;
    inputs.latestTargetObservation = current.latestTargetObservation;
    inputs.pose3d = current.pose3d;
    inputs.megatagPoseEstimate = current.megatagPoseEstimate;
    inputs.fiducialObservations = current.fiducialObservations;
    inputs.standardDeviations = current.standardDeviations;
    inputs.poseObservations = current.poseObservations;
    inputs.tagIds = current.tagIds;
    inputs.residualTranslationMeters = current.residualTranslationMeters;
    inputs.rejectReason = current.rejectReason;
    inputs.limelightProfile = LimelightProfile.LL4.name();
    inputs.limelightProfileSource = "SIM";
  }

  private Packet capturePacket(long nowNanos) {
    Pose2d robotPose = simulation.getPose();
    Pose3d robotPose3d = simulation.getPose3d();
    Transform3d robotToCamera = cameraConstants.robotToCamera();
    Pose3d cameraPose = robotPose3d.transformBy(robotToCamera);

    List<TagVisibility> visibleTags = new ArrayList<>();
    for (var tag : GlobalConstants.FieldConstants.defaultAprilTagType.getLayout().getTags()) {
      Pose3d tagPose = tag.pose;
      double dx = tagPose.getX() - cameraPose.getX();
      double dy = tagPose.getY() - cameraPose.getY();
      double dz = tagPose.getZ() - cameraPose.getZ();
      double distance = Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
      if (distance > Math.max(2.5, cameraConstants.cameraType().noisyDistance)) {
        continue;
      }
      double cameraYaw = cameraPose.getRotation().getZ();
      double cameraPitch = cameraPose.getRotation().getY();
      double yaw = Math.atan2(dy, dx) - cameraYaw;
      double planarDistance = Math.hypot(dx, dy);
      double pitch = Math.atan2(dz, planarDistance) - cameraPitch;
      if (Math.abs(Rotation2d.fromRadians(yaw).getRadians()) > (HORIZONTAL_FOV_RADIANS * 0.5)) {
        continue;
      }
      if (Math.abs(Rotation2d.fromRadians(pitch).getRadians()) > (VERTICAL_FOV_RADIANS * 0.5)) {
        continue;
      }
      if (isOccluded(cameraPose, tagPose)) {
        continue;
      }
      visibleTags.add(new TagVisibility(tag.ID, tagPose, distance, yaw, pitch));
    }
    visibleTags.sort(Comparator.comparingDouble(TagVisibility::distanceMeters));
    if (visibleTags.isEmpty()) {
      return Packet.empty();
    }

    int[] tagIds = visibleTags.stream().mapToInt(TagVisibility::tagId).toArray();
    FiducialObservation[] observations =
        visibleTags.stream()
            .limit(4)
            .map(
                visible ->
                    new FiducialObservation(
                        visible.tagId(),
                        visible.yawRadians() / (HORIZONTAL_FOV_RADIANS * 0.5),
                        visible.pitchRadians() / (VERTICAL_FOV_RADIANS * 0.5),
                        0.05,
                        Math.max(0.01, 1.0 / Math.max(visible.distanceMeters(), 0.5)),
                        visible.distanceMeters()))
            .toArray(FiducialObservation[]::new);

    TagVisibility best = visibleTags.get(0);
    double rangeScale = Math.max(0.5, best.distanceMeters());
    double yawScale =
        1.0
            + (Math.abs(best.yawRadians()) / (HORIZONTAL_FOV_RADIANS * 0.5))
            + (Math.abs(best.pitchRadians()) / (VERTICAL_FOV_RADIANS * 0.5));
    double xyNoiseMeters = 0.008 * rangeScale * yawScale;
    double thetaNoiseRadians = Math.toRadians(0.25 + (best.distanceMeters() * 0.4)) * yawScale;

    Pose2d noisyRobotPose =
        new Pose2d(
            robotPose.getX() + (xNoise.sample() * xyNoiseMeters),
            robotPose.getY() + (yNoise.sample() * xyNoiseMeters),
            robotPose
                .getRotation()
                .plus(Rotation2d.fromRadians(thetaNoise.sample() * thetaNoiseRadians)));
    double timestampSeconds = nowNanos / 1_000_000_000.0;
    double avgTagDist =
        visibleTags.stream().mapToDouble(TagVisibility::distanceMeters).average().orElse(0.0);
    MegatagPoseEstimate megatagPoseEstimate =
        new MegatagPoseEstimate(
            noisyRobotPose,
            timestampSeconds,
            LATENCY_NANOS / 1_000_000_000.0,
            Math.max(0.02, 1.0 / Math.max(avgTagDist, 0.5)),
            avgTagDist,
            Math.min(1.0, 1.5 / Math.max(avgTagDist, 0.75)),
            tagIds,
            Math.hypot(xNoise.sample(), yNoise.sample()));
    PoseObservation[] poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestampSeconds, new Pose3d(noisyRobotPose), 0.05, tagIds.length, avgTagDist)
        };
    double[] standardDeviations = {xyNoiseMeters, xyNoiseMeters, thetaNoiseRadians};
    return new Packet(
        true,
        tagIds.length,
        new TargetObservation(
            Rotation2d.fromRadians(best.yawRadians()), Rotation2d.fromRadians(best.pitchRadians())),
        new Pose3d(noisyRobotPose),
        megatagPoseEstimate,
        observations,
        standardDeviations,
        poseObservations,
        tagIds,
        megatagPoseEstimate.residualTranslation(),
        RejectReason.ACCEPTED);
  }

  private boolean isOccluded(Pose3d cameraPose, Pose3d tagPose) {
    double cx = cameraPose.getX();
    double cy = cameraPose.getY();
    double cz = cameraPose.getZ();
    double tx = tagPose.getX();
    double ty = tagPose.getY();
    double tz = tagPose.getZ();
    for (var body : simulation.getBodies()) {
      if ("chassis".equals(body.name()) || body.name().startsWith("module-")) {
        continue;
      }
      if (body.shape().type()
          != org.Griffins1884.frc2026.simulation.engine.CollisionShape.ShapeType.BOX) {
        continue;
      }
      var aabb = collisionSystem.computeAabb(body);
      if (segmentIntersectsAabb(
          cx,
          cy,
          cz,
          tx,
          ty,
          tz,
          aabb.min().x(),
          aabb.max().x(),
          aabb.min().y(),
          aabb.max().y(),
          aabb.min().z(),
          aabb.max().z())) {
        return true;
      }
    }
    return false;
  }

  private boolean segmentIntersectsAabb(
      double x0,
      double y0,
      double z0,
      double x1,
      double y1,
      double z1,
      double minX,
      double maxX,
      double minY,
      double maxY,
      double minZ,
      double maxZ) {
    double tMin = 0.0;
    double tMax = 1.0;
    double[] origin = {x0, y0, z0};
    double[] direction = {x1 - x0, y1 - y0, z1 - z0};
    double[] mins = {minX, minY, minZ};
    double[] maxs = {maxX, maxY, maxZ};
    for (int axis = 0; axis < 3; axis++) {
      double dir = direction[axis];
      if (Math.abs(dir) < 1e-9) {
        if (origin[axis] < mins[axis] || origin[axis] > maxs[axis]) {
          return false;
        }
        continue;
      }
      double invDir = 1.0 / dir;
      double t1 = (mins[axis] - origin[axis]) * invDir;
      double t2 = (maxs[axis] - origin[axis]) * invDir;
      if (t1 > t2) {
        double tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = Math.max(tMin, t1);
      tMax = Math.min(tMax, t2);
      if (tMin > tMax) {
        return false;
      }
    }
    return tMax > 0.0 && tMin < 1.0;
  }

  private record TagVisibility(
      int tagId, Pose3d pose, double distanceMeters, double yawRadians, double pitchRadians) {}

  private record Packet(
      boolean seesTarget,
      int megatagCount,
      TargetObservation latestTargetObservation,
      Pose3d pose3d,
      MegatagPoseEstimate megatagPoseEstimate,
      FiducialObservation[] fiducialObservations,
      double[] standardDeviations,
      PoseObservation[] poseObservations,
      int[] tagIds,
      double residualTranslationMeters,
      RejectReason rejectReason) {
    private static Packet empty() {
      return new Packet(
          false,
          0,
          new TargetObservation(new Rotation2d(), new Rotation2d()),
          null,
          null,
          new FiducialObservation[0],
          new double[0],
          new PoseObservation[0],
          new int[0],
          0.0,
          RejectReason.NO_TAGS);
    }
  }
}
