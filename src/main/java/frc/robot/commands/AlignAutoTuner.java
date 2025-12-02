// package frc.robot.commands;
//
// import static frc.robot.commands.AlignConstants.*;
//
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.GlobalConstants;
// import frc.robot.commands.AlignConstants.AlignGains;
// import frc.robot.commands.AlignConstants.FeedforwardGains;
// import frc.robot.commands.AlignConstants.PIDGains;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import java.util.ArrayList;
// import java.util.Comparator;
// import java.util.List;
// import java.util.Set;
// import java.util.function.Consumer;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;
//
// @Deprecated
// public final class AlignAutoTuner {
//  private static final Translation2d CENTRAL_REEF_CENTER = calculateReefCenter();
//
//  private AlignAutoTuner() {}
//
//  /** Runs an aggressive full-field alignment auto-tune across reefs, sources, and processor. */
//  public static Command autoTuneAlignGainsFullField(SwerveSubsystem drive) {
//    List<AlignTarget> targets =
//        List.of(
//            new AlignTarget("Reef 1", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_1.getPose()),
//            new AlignTarget("Reef 2", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_2.getPose()),
//            new AlignTarget("Reef 3", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_3.getPose()),
//            new AlignTarget("Reef 4", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_4.getPose()),
//            new AlignTarget("Reef 5", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_5.getPose()),
//            new AlignTarget("Reef 6", () ->
// GlobalConstants.FieldMap.Coordinates.REEF_6.getPose()),
//            new AlignTarget(
//                "Left Source",
//                () -> GlobalConstants.FieldMap.Coordinates.LEFT_CORAL_STATION.getPose()),
//            new AlignTarget(
//                "Right Source",
//                () -> GlobalConstants.FieldMap.Coordinates.RIGHT_CORAL_STATION.getPose()),
//            new AlignTarget(
//                "Processor", () -> GlobalConstants.FieldMap.Coordinates.PROCESSOR.getPose()));
//
//    return Commands.defer(
//        () -> {
//          List<AlignTuningTrial> trials = new ArrayList<>();
//          List<Command> iterations = new ArrayList<>();
//          for (AlignTarget target : targets) {
//            iterations.add(
//                runAlignTuningIteration(drive, target.name(), target.poseSupplier(), trials,
// true));
//          }
//          Command[] iterationArray = iterations.toArray(new Command[0]);
//          return Commands.sequence(
//              Commands.runOnce(() -> DriveCommands.setAlignContext("AutoTune", "FullField")),
//              Commands.runOnce(() -> DriveCommands.setAlignGains(DEFAULT_ALIGN_GAINS)),
//              Commands.sequence(iterationArray),
//              Commands.runOnce(
//                  () -> {
//                    AlignGains chosen = pickBestAlignGains(trials);
//                    DriveCommands.setAlignGains(chosen);
//                    Logger.recordOutput(
//                        "Autonomy/AlignAutoTuner/RecommendedGains", flattenGains(chosen));
//                  }),
//              Commands.runOnce(DriveCommands::clearAlignTelemetry));
//        },
//        Set.of(drive));
//  }
//
//  /** Auto-tunes alignment feedforward and PID by running multiple align attempts. */
//  public static Command autoTuneAlignGains(SwerveSubsystem drive, Supplier<Pose2d> targetSupplier)
// {
//    return Commands.defer(
//        () -> {
//          List<AlignTuningTrial> trials = new ArrayList<>();
//          List<Command> iterations = new ArrayList<>();
//          for (int i = 0; i < ALIGN_TUNER_TRIAL_COUNT; i++) {
//            iterations.add(
//                runAlignTuningIteration(drive, "Custom Target", targetSupplier, trials, false));
//          }
//          Command[] iterationArray = iterations.toArray(new Command[0]);
//          return Commands.sequence(
//              Commands.runOnce(() -> DriveCommands.setAlignContext("AutoTune", "Align")),
//              Commands.runOnce(() -> DriveCommands.setAlignGains(DEFAULT_ALIGN_GAINS)),
//              Commands.sequence(iterationArray),
//              Commands.runOnce(
//                  () -> {
//                    AlignGains chosen = pickBestAlignGains(trials);
//                    DriveCommands.setAlignGains(chosen);
//                    Logger.recordOutput(
//                        "Autonomy/AlignAutoTuner/RecommendedGains", flattenGains(chosen));
//                  }),
//              Commands.runOnce(DriveCommands::clearAlignTelemetry));
//        },
//        Set.of(drive));
//  }
//
//  private static Command runAlignTuningIteration(
//      SwerveSubsystem drive,
//      String targetName,
//      Supplier<Pose2d> targetSupplier,
//      List<AlignTuningTrial> trials,
//      boolean avoidCentralReef) {
//    return Commands.defer(
//        () -> {
//          AlignTuningTrial trial = new AlignTuningTrial();
//          trial.targetName = targetName;
//          trial.gainsUsed = DriveCommands.getAlignGains();
//          Timer timer = new Timer();
//          Consumer<DriveCommands.AlignLoopTelemetry> telemetryConsumer =
//              telemetry -> {
//                double distance = telemetry.errorPose().getTranslation().getNorm();
//                if (trial.initialDistance < 1e-3) {
//                  trial.initialDistance = distance;
//                }
//                trial.maxDistance = Math.max(trial.maxDistance, distance);
//                trial.finalDistance = distance;
//                trial.maxRotationErrorDeg =
//                    Math.max(
//                        trial.maxRotationErrorDeg,
//                        Math.abs(telemetry.errorPose().getRotation().getDegrees()));
//                var speeds = telemetry.fieldRelativeSpeeds();
//                double fieldSpeed = Math.hypot(speeds.vxMetersPerSecond,
// speeds.vyMetersPerSecond);
//                trial.maxFieldSpeed = Math.max(trial.maxFieldSpeed, fieldSpeed);
//              };
//
//          Command detour =
//              avoidCentralReef ? createAlignTunerBypass(drive, targetSupplier) : Commands.none();
//
//          Command tuneAlign =
//              DriveCommands.holonomicAlignCommand(
//                      drive,
//                      targetSupplier,
//                      () -> 0.0,
//                      false,
//                      true,
//                      () -> trial.gainsUsed,
//                      telemetryConsumer)
//                  .beforeStarting(timer::restart);
//
//          Command timeoutFlag =
//              Commands.sequence(
//                  Commands.waitSeconds(ALIGN_TUNER_TIMEOUT_SECS),
//                  Commands.runOnce(() -> trial.timedOut = true));
//
//          return Commands.sequence(
//                  createAlignTunerKickback(drive, targetSupplier),
//                  detour,
//                  Commands.race(tuneAlign, timeoutFlag))
//              .finallyDo(
//                  () -> {
//                    timer.stop();
//                    trial.settleTimeSec = timer.get();
//                    trials.add(trial);
//                    AlignGains updated = refineAlignGains(trial.gainsUsed, trial);
//                    DriveCommands.setAlignGains(updated);
//                    Logger.recordOutput("Autonomy/AlignAutoTuner/LastTarget", trial.targetName);
//                    Logger.recordOutput(
//                        "Autonomy/AlignAutoTuner/LastTrialMetrics",
//                        new double[] {
//                          trial.initialDistance,
//                          trial.finalDistance,
//                          trial.maxFieldSpeed,
//                          trial.settleTimeSec,
//                          trial.maxRotationErrorDeg,
//                          trial.timedOut ? 1.0 : 0.0
//                        });
//                    Logger.recordOutput(
//                        "Autonomy/AlignAutoTuner/LastTrialSec", trial.settleTimeSec);
//                  });
//        },
//        Set.of(drive));
//  }
//
//  private static Command createAlignTunerKickback(
//      SwerveSubsystem drive, Supplier<Pose2d> targetSupplier) {
//    return Commands.sequence(
//        Commands.deadline(
//            Commands.waitSeconds(ALIGN_TUNER_KICKBACK_TIME),
//            Commands.run(
//                () -> {
//                  Pose2d target = targetSupplier.get();
//                  Rotation2d awayHeading = target.getRotation().plus(Rotation2d.k180deg);
//                  Translation2d retreatVector =
//                      new Translation2d(ALIGN_TUNER_KICKBACK_SPEED, 0.0).rotateBy(awayHeading);
//                  ChassisSpeeds speeds =
//                      ChassisSpeeds.fromFieldRelativeSpeeds(
//                          retreatVector.getX(), retreatVector.getY(), 0.0, drive.getRotation());
//                  drive.runVelocity(speeds);
//                },
//                drive)),
//        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds())));
//  }
//
//  private static Command createAlignTunerBypass(
//      SwerveSubsystem drive, Supplier<Pose2d> targetSupplier) {
//    return Commands.defer(
//        () -> {
//          Pose2d start = drive.getPose();
//          Pose2d target = targetSupplier.get();
//          Pose2d waypoint = calculateReefBypassWaypoint(start, target);
//          if (waypoint == null) {
//            return Commands.none();
//          }
//          Supplier<Pose2d> waypointSupplier = () -> waypoint;
//          return DriveCommands.holonomicAlignCommand(
//                  drive, waypointSupplier, () -> 0.0, false, true)
//              .withTimeout(ALIGN_BYPASS_TIMEOUT_SECS);
//        },
//        Set.of(drive));
//  }
//
//  private static AlignGains refineAlignGains(AlignGains current, AlignTuningTrial trial) {
//    AlignGains updated = current;
//    if (trial.timedOut) {
//      updated = updated.withFeedforward(current.feedforward() * 1.2);
//      updated = updated.withTranslationKp(current.translationKp() * 1.1);
//      return clampAlignGains(updated);
//    }
//
//    double speedTarget = ALIGN_MAX_TRANSLATIONAL_SPEED * 0.9;
//    if (trial.maxFieldSpeed < speedTarget) {
//      updated = updated.withFeedforward(current.feedforward() * 1.12);
//    }
//
//    if (trial.settleTimeSec > 1.2 || trial.finalDistance > 0.03) {
//      updated = updated.withTranslationKp(current.translationKp() * 1.08);
//    } else if (trial.maxDistance > trial.initialDistance * 1.15) {
//      updated = updated.withTranslationKp(current.translationKp() * 0.92);
//    }
//
//    if (trial.maxRotationErrorDeg > 7.0) {
//      updated = updated.withRotationKp(current.rotationKp() * 1.05);
//    }
//
//    return clampAlignGains(updated);
//  }
//
//  private static AlignGains clampAlignGains(AlignGains gains) {
//    FeedforwardGains ff =
//        gains.feedforwardGains().withKv(MathUtil.clamp(gains.feedforward(), 0.5, 7.0));
//    PIDGains x =
//        new PIDGains(
//            MathUtil.clamp(gains.xGains().kP(), 1.0, 10.0),
//            gains.xGains().kI(),
//            gains.xGains().kD());
//    PIDGains y =
//        new PIDGains(
//            MathUtil.clamp(gains.yGains().kP(), 1.0, 10.0),
//            gains.yGains().kI(),
//            gains.yGains().kD());
//    PIDGains theta =
//        new PIDGains(
//            MathUtil.clamp(gains.thetaGains().kP(), 2.0, 12.0),
//            gains.thetaGains().kI(),
//            gains.thetaGains().kD());
//    return new AlignGains(x, y, theta, ff);
//  }
//
//  private static AlignGains pickBestAlignGains(List<AlignTuningTrial> trials) {
//    return trials.stream()
//        .filter(trial -> !trial.timedOut)
//        .min(Comparator.comparingDouble(trial -> trial.settleTimeSec + trial.finalDistance * 5))
//        .map(trial -> trial.gainsUsed)
//        .orElse(DriveCommands.getCurrentAlignGains());
//  }
//
//  private static Pose2d calculateReefBypassWaypoint(Pose2d start, Pose2d target) {
//    if (!pathCrossesCentralReef(start, target)) {
//      return null;
//    }
//    Translation2d startT = start.getTranslation();
//    Translation2d targetT = target.getTranslation();
//    Translation2d delta = targetT.minus(startT);
//    double length = delta.getNorm();
//    if (length < 1e-3) {
//      return null;
//    }
//    Translation2d direction = new Translation2d(delta.getX() / length, delta.getY() / length);
//    Translation2d normal = new Translation2d(-direction.getY(), direction.getX());
//    Translation2d mid = startT.plus(delta.times(0.5));
//    double clearance = CENTRAL_REEF_RADIUS_METERS + CENTRAL_REEF_MARGIN_METERS;
//    Translation2d candidateA = mid.plus(normal.times(clearance));
//    Translation2d candidateB = mid.minus(normal.times(clearance));
//    double distA = candidateA.getDistance(CENTRAL_REEF_CENTER);
//    double distB = candidateB.getDistance(CENTRAL_REEF_CENTER);
//    Translation2d chosen = distA > distB ? candidateA : candidateB;
//    return new Pose2d(chosen, target.getRotation());
//  }
//
//  private static boolean pathCrossesCentralReef(Pose2d start, Pose2d target) {
//    double distance =
//        distanceFromPointToSegment(
//            CENTRAL_REEF_CENTER, start.getTranslation(), target.getTranslation());
//    return distance < CENTRAL_REEF_RADIUS_METERS + CENTRAL_REEF_MARGIN_METERS;
//  }
//
//  private static double distanceFromPointToSegment(
//      Translation2d point, Translation2d segmentStart, Translation2d segmentEnd) {
//    Translation2d seg = segmentEnd.minus(segmentStart);
//    double segLengthSquared = Math.pow(seg.getNorm(), 2);
//    if (segLengthSquared < 1e-6) {
//      return point.getDistance(segmentStart);
//    }
//    Translation2d startToPoint = point.minus(segmentStart);
//    double t =
//        (startToPoint.getX() * seg.getX() + startToPoint.getY() * seg.getY()) / segLengthSquared;
//    t = MathUtil.clamp(t, 0.0, 1.0);
//    Translation2d projection =
//        new Translation2d(
//            segmentStart.getX() + t * seg.getX(), segmentStart.getY() + t * seg.getY());
//    return point.getDistance(projection);
//  }
//
//  private static Translation2d calculateReefCenter() {
//    Pose2d[] faces =
//        new Pose2d[] {
//          GlobalConstants.FieldMap.Coordinates.REEF_1.getPose(),
//          GlobalConstants.FieldMap.Coordinates.REEF_2.getPose(),
//          GlobalConstants.FieldMap.Coordinates.REEF_3.getPose(),
//          GlobalConstants.FieldMap.Coordinates.REEF_4.getPose(),
//          GlobalConstants.FieldMap.Coordinates.REEF_5.getPose(),
//          GlobalConstants.FieldMap.Coordinates.REEF_6.getPose()
//        };
//    double x = 0.0;
//    double y = 0.0;
//    for (Pose2d pose : faces) {
//      x += pose.getX();
//      y += pose.getY();
//    }
//    return new Translation2d(x / faces.length, y / faces.length);
//  }
//
//  private static double[] flattenGains(AlignGains gains) {
//    return new double[] {
//      gains.xGains().kP(),
//      gains.xGains().kI(),
//      gains.xGains().kD(),
//      gains.yGains().kP(),
//      gains.yGains().kI(),
//      gains.yGains().kD(),
//      gains.thetaGains().kP(),
//      gains.thetaGains().kI(),
//      gains.thetaGains().kD(),
//      gains.feedforwardGains().kV(),
//      gains.feedforwardGains().deadbandMeters(),
//      gains.feedforwardGains().maxSpeedMetersPerSecond()
//    };
//  }
//
//  private record AlignTarget(String name, Supplier<Pose2d> poseSupplier) {}
//
//  private static class AlignTuningTrial {
//    String targetName = "";
//    AlignGains gainsUsed = DEFAULT_ALIGN_GAINS;
//    double initialDistance = 0.0;
//    double maxDistance = 0.0;
//    double finalDistance = 0.0;
//    double maxRotationErrorDeg = 0.0;
//    double settleTimeSec = 0.0;
//    double maxFieldSpeed = 0.0;
//    boolean timedOut = false;
//  }
// }
