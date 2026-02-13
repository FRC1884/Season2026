package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.Griffins1884.frc2026.GlobalConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class VisionArbitrationTest {
  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    GlobalConstants.TUNING_MODE = false;
  }

  @Test
  void fusesWhenAllCandidatesAreConsistent() {
    double now = Timer.getFPGATimestamp();
    RecordingConsumer consumer = new RecordingConsumer();
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO(
                "camA",
                makeInputs(
                    new Pose2d(2.00, 2.00, Rotation2d.fromDegrees(0.0)),
                    now,
                    2,
                    1.0,
                    0.25,
                    0.7,
                    0.7,
                    4.0)),
            new StubVisionIO(
                "camB",
                makeInputs(
                    new Pose2d(2.08, 2.03, Rotation2d.fromDegrees(3.0)),
                    now,
                    2,
                    1.0,
                    0.25,
                    0.7,
                    0.7,
                    4.0)));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d fused = consumer.acceptedPoses.get(0);
    assertTrue(fused.getX() > 2.00 && fused.getX() < 2.08);
    assertTrue(fused.getY() > 2.00 && fused.getY() < 2.03);
    assertTrue(fused.getRotation().getDegrees() > 0.0 && fused.getRotation().getDegrees() < 3.0);
  }

  @Test
  void fallsBackToBestSingleWhenCandidatesConflict() {
    double now = Timer.getFPGATimestamp();
    RecordingConsumer consumer = new RecordingConsumer();
    Pose2d bestPose = new Pose2d(1.20, 1.10, Rotation2d.fromDegrees(2.0));
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO("good", makeInputs(bestPose, now, 2, 1.0, 0.3, 0.45, 0.45, 3.0)),
            new StubVisionIO(
                "bad",
                makeInputs(
                    new Pose2d(4.0, 5.0, Rotation2d.fromDegrees(90.0)),
                    now,
                    1,
                    0.35,
                    0.02,
                    2.5,
                    2.5,
                    20.0)));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d selected = consumer.acceptedPoses.get(0);
    assertEquals(bestPose.getX(), selected.getX(), 1e-9);
    assertEquals(bestPose.getY(), selected.getY(), 1e-9);
    assertEquals(bestPose.getRotation().getDegrees(), selected.getRotation().getDegrees(), 1e-9);
  }

  @Test
  void usesSingleCandidateWhenOnlyOneExists() {
    double now = Timer.getFPGATimestamp();
    RecordingConsumer consumer = new RecordingConsumer();
    Pose2d soloPose = new Pose2d(3.1, 2.2, Rotation2d.fromDegrees(-7.0));
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO("solo", makeInputs(soloPose, now, 2, 1.0, 0.25, 0.8, 0.8, 5.0)),
            new StubVisionIO("empty", makeDisconnectedInputs()));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d selected = consumer.acceptedPoses.get(0);
    assertEquals(soloPose.getX(), selected.getX(), 1e-9);
    assertEquals(soloPose.getY(), selected.getY(), 1e-9);
    assertEquals(soloPose.getRotation().getDegrees(), selected.getRotation().getDegrees(), 1e-9);
  }

  @Test
  void emitsNoMeasurementWhenNoCandidatesValid() {
    RecordingConsumer consumer = new RecordingConsumer();
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO("camA", makeDisconnectedInputs()),
            new StubVisionIO("camB", makeDisconnectedInputs()));

    vision.periodic();

    assertEquals(0, consumer.acceptedPoses.size());
  }

  @Test
  void disagreementAcrossThreeCamerasStillFallsBackToSingle() {
    double now = Timer.getFPGATimestamp();
    RecordingConsumer consumer = new RecordingConsumer();
    Pose2d bestPose = new Pose2d(2.00, 1.00, Rotation2d.fromDegrees(1.0));
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO("camA", makeInputs(bestPose, now, 2, 1.0, 0.3, 0.4, 0.4, 3.0)),
            new StubVisionIO(
                "camB",
                makeInputs(
                    new Pose2d(2.07, 1.04, Rotation2d.fromDegrees(2.5)),
                    now,
                    2,
                    0.9,
                    0.2,
                    0.8,
                    0.8,
                    6.0)),
            new StubVisionIO(
                "camC",
                makeInputs(
                    new Pose2d(5.0, 1.0, Rotation2d.fromDegrees(-40.0)),
                    now,
                    2,
                    0.5,
                    0.05,
                    1.8,
                    1.8,
                    12.0)));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d selected = consumer.acceptedPoses.get(0);
    assertEquals(bestPose.getX(), selected.getX(), 1e-9);
    assertEquals(bestPose.getY(), selected.getY(), 1e-9);
    assertEquals(bestPose.getRotation().getDegrees(), selected.getRotation().getDegrees(), 1e-9);
  }

  @Test
  void fusesAtConsistencyThresholdBoundaries() {
    double now = Timer.getFPGATimestamp();
    double maxDeltaMeters = AprilTagVisionConstants.getLimelightMultiCamMaxDeltaMeters();
    double maxDeltaDeg = AprilTagVisionConstants.getLimelightMultiCamMaxDeltaDeg();
    RecordingConsumer consumer = new RecordingConsumer();
    Vision vision =
        new Vision(
            consumer,
            Pose2d::new,
            () -> 0.0,
            new StubVisionIO(
                "camA",
                makeInputs(
                    new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0.0)),
                    now,
                    2,
                    1.0,
                    0.3,
                    0.7,
                    0.7,
                    4.0)),
            new StubVisionIO(
                "camB",
                makeInputs(
                    new Pose2d(1.0 + maxDeltaMeters, 1.0, Rotation2d.fromDegrees(maxDeltaDeg)),
                    now,
                    2,
                    1.0,
                    0.3,
                    0.7,
                    0.7,
                    4.0)));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d fused = consumer.acceptedPoses.get(0);
    assertTrue(fused.getX() > 1.0 && fused.getX() < 1.0 + maxDeltaMeters);
    assertTrue(
        fused.getRotation().getDegrees() > 0.0 && fused.getRotation().getDegrees() < maxDeltaDeg);
  }

  @Test
  void usesOdometryAlignmentForCrossTimestampConsistency() {
    RecordingConsumer consumer = new RecordingConsumer();
    AtomicReference<Pose2d> odomPose =
        new AtomicReference<>(new Pose2d(0.0, 0.0, new Rotation2d()));
    StubVisionIO camA = new StubVisionIO("camA", makeDisconnectedInputs());
    StubVisionIO camB = new StubVisionIO("camB", makeDisconnectedInputs());
    Vision vision = new Vision(consumer, odomPose::get, () -> 0.0, camA, camB);

    vision.periodic();
    double tsOld = Timer.getFPGATimestamp();
    SimHooks.stepTiming(0.02);

    odomPose.set(new Pose2d(1.0, 0.0, new Rotation2d()));
    vision.periodic();
    double tsNew = Timer.getFPGATimestamp();
    SimHooks.stepTiming(0.02);

    camA.setSource(
        makeInputs(
            new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0.0)), tsOld, 2, 1.0, 0.3, 0.6, 0.6, 4.0));
    camB.setSource(
        makeInputs(
            new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0.0)), tsNew, 2, 1.0, 0.3, 0.6, 0.6, 4.0));

    vision.periodic();

    assertEquals(1, consumer.acceptedPoses.size());
    Pose2d selected = consumer.acceptedPoses.get(0);
    assertTrue(selected.getX() > 2.5);
  }

  private static VisionIO.VisionIOInputs makeInputs(
      Pose2d pose,
      double timestamp,
      int tagCount,
      double quality,
      double avgTagArea,
      double mt2XStd,
      double mt2YStd,
      double mt2YawStdDeg) {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    inputs.connected = true;
    inputs.seesTarget = true;
    inputs.megatagCount = tagCount;
    inputs.latestTargetObservation =
        new VisionIO.TargetObservation(new Rotation2d(), new Rotation2d());
    int[] tagIds = new int[tagCount];
    for (int i = 0; i < tagCount; i++) {
      tagIds[i] = i + 1;
    }
    inputs.tagIds = tagIds.clone();
    inputs.megatagPoseEstimate =
        new MegatagPoseEstimate(pose, timestamp, 0.0, avgTagArea, quality, tagIds);
    inputs.standardDeviations =
        new double[] {
          1.2, 1.2, Math.toRadians(6.0), mt2XStd, mt2YStd, Math.toRadians(mt2YawStdDeg)
        };
    inputs.fiducialObservations = new FiducialObservation[tagCount];
    for (int i = 0; i < tagCount; i++) {
      inputs.fiducialObservations[i] = new FiducialObservation(i + 1, 0.0, 0.0, 0.1, avgTagArea);
    }
    return inputs;
  }

  private static VisionIO.VisionIOInputs makeDisconnectedInputs() {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    inputs.connected = false;
    inputs.seesTarget = false;
    inputs.megatagCount = 0;
    inputs.standardDeviations = AprilTagVisionConstants.getLimelightStandardDeviations();
    return inputs;
  }

  private static class RecordingConsumer implements Vision.VisionConsumer {
    private final List<Pose2d> acceptedPoses = new ArrayList<>();

    @Override
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1>
            visionMeasurementStdDevs) {
      acceptedPoses.add(visionRobotPoseMeters);
    }
  }

  private static class StubVisionIO implements VisionIO {
    private final CameraConstants cameraConstants;
    private VisionIOInputs source;

    private StubVisionIO(String cameraName, VisionIOInputs source) {
      this.cameraConstants =
          new CameraConstants(cameraName, new Transform3d(), CameraType.LIMELIGHT);
      this.source = source;
    }

    private void setSource(VisionIOInputs source) {
      this.source = source;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = source.connected;
      inputs.seesTarget = source.seesTarget;
      inputs.megatagCount = source.megatagCount;
      inputs.latestTargetObservation = source.latestTargetObservation;
      inputs.pose3d = source.pose3d;
      inputs.megatagPoseEstimate = source.megatagPoseEstimate;
      inputs.fiducialObservations = source.fiducialObservations.clone();
      inputs.standardDeviations = source.standardDeviations.clone();
      inputs.poseObservations = source.poseObservations.clone();
      inputs.tagIds = source.tagIds.clone();
    }

    @Override
    public CameraConstants getCameraConstants() {
      return cameraConstants;
    }
  }
}
