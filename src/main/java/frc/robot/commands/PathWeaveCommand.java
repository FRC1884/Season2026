package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.RotationalAllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathWeaveCommand extends Command {
  public enum edges {
    CORNER_1(new Pose2d(3, 4.9, new Rotation2d(-Math.PI / 6))),
    CORNER_2(new Pose2d(3, 3.15, new Rotation2d(Math.PI / 6))),
    CORNER_3(new Pose2d(4.5, 2.3, new Rotation2d((Math.PI) / 2))),
    CORNER_4(new Pose2d(6, 3.15, new Rotation2d((5 * Math.PI) / 6))),
    CORNER_5(new Pose2d(6, 4.9, new Rotation2d(-(5 * Math.PI) / 6))),
    CORNER_6(new Pose2d(4.5, 5.8, new Rotation2d((-Math.PI) / 2)));

    private final Pose2d bluePose;

    edges(Pose2d pose) {
      this.bluePose = pose;
    }

    public Pose2d getPose() {
      return RotationalAllianceFlipUtil.apply(bluePose);
    }
  }

  private final Pose2d target;
  private Supplier<Pose2d> nextTarget;
  private final SwerveSubsystem drive;
  private Command autoAlign;
  private final List<Pose2d> plannedPath = new ArrayList<>();
  private int currentIndex = 0;
  private Pose2d lastEdge = edges.CORNER_1.getPose();
  private double currentLegHandoffDistance = 0.0;
  private static final double MIN_HANDOFF_DISTANCE = 0.25; // m
  private static final double MAX_HANDOFF_DISTANCE = 0.6; // m
  private static final Translation2d BLUE_REEF_CENTER = calculateBlueReefCenter();
  private static final double ANGLE_MATCH_EPSILON = Math.toRadians(5.0);
  private static final double ANGLE_DIVERGENCE_EPSILON = Math.toRadians(1.0);
  private static final double TARGET_PASS_THROUGH_MARGIN = 0.2; // m

  public PathWeaveCommand(SwerveSubsystem drive, Pose2d target) {
    this.drive = drive;
    this.target = target;
    this.nextTarget = null;
    this.autoAlign = null;
    if (target != null) {
      rebuildPlannedPath(drive.getPose());
    }
  }

  private double distanceBetween(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  private static Translation2d calculateBlueReefCenter() {
    double sumX = 0.0;
    double sumY = 0.0;
    for (edges e : edges.values()) {
      sumX += e.bluePose.getX();
      sumY += e.bluePose.getY();
    }
    return new Translation2d(sumX / edges.values().length, sumY / edges.values().length);
  }

  private Translation2d getReefCenter() {
    return RotationalAllianceFlipUtil.apply(BLUE_REEF_CENTER);
  }

  private double angleFromReefCenter(Pose2d pose) {
    Translation2d vec = pose.getTranslation().minus(getReefCenter());
    return Math.atan2(vec.getY(), vec.getX());
  }

  private double smallestAngleDiff(double a, double b) {
    double diff = a - b;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return Math.abs(diff);
  }

  private double distancePointToSegment(
      Translation2d point, Translation2d segStart, Translation2d segEnd) {
    Translation2d segVec = segEnd.minus(segStart);
    double segLenSq = segVec.getX() * segVec.getX() + segVec.getY() * segVec.getY();
    if (segLenSq < 1e-9) {
      return point.getDistance(segStart);
    }
    double t =
        ((point.getX() - segStart.getX()) * segVec.getX()
                + (point.getY() - segStart.getY()) * segVec.getY())
            / segLenSq;
    t = Math.max(0.0, Math.min(1.0, t));
    Translation2d projection =
        new Translation2d(segStart.getX() + t * segVec.getX(), segStart.getY() + t * segVec.getY());
    return point.getDistance(projection);
  }

  private boolean segmentHitsReef(Pose2d start, Pose2d end) {
    double clearance =
        AlignConstants.CENTRAL_REEF_RADIUS_METERS + AlignConstants.CENTRAL_REEF_MARGIN_METERS;
    double dist =
        distancePointToSegment(getReefCenter(), start.getTranslation(), end.getTranslation());
    return dist <= clearance;
  }

  private edges findClosestEdge(Pose2d pose) {
    edges best = edges.CORNER_1;
    double bestDistance = Double.MAX_VALUE;
    for (edges e : edges.values()) {
      double dist = distanceBetween(pose, e.getPose());
      if (dist < bestDistance) {
        bestDistance = dist;
        best = e;
      }
    }
    return best;
  }

  private edges findClosestEdgeByAngle(Pose2d pose) {
    double targetAngle = angleFromReefCenter(pose);
    edges best = edges.CORNER_1;
    double bestAngleDiff = Double.MAX_VALUE;
    double bestDistance = Double.MAX_VALUE;

    for (edges e : edges.values()) {
      double angleDiff = smallestAngleDiff(angleFromReefCenter(e.getPose()), targetAngle);
      double dist = distanceBetween(pose, e.getPose());
      if (angleDiff < bestAngleDiff - 1e-6
          || (Math.abs(angleDiff - bestAngleDiff) <= 1e-6 && dist < bestDistance)) {
        bestAngleDiff = angleDiff;
        bestDistance = dist;
        best = e;
      }
    }
    return best;
  }

  private List<Pose2d> buildEdgeWalk(edges startEdge, edges targetEdge, boolean clockwise) {
    List<Pose2d> walk = new ArrayList<>();
    edges[] ordered = edges.values();
    int step = clockwise ? 1 : -1;
    int idx = startEdge.ordinal();
    int targetIdx = targetEdge.ordinal();

    walk.add(ordered[idx].getPose());
    while (idx != targetIdx) {
      idx = (idx + step + ordered.length) % ordered.length;
      walk.add(ordered[idx].getPose());
    }
    return walk;
  }

  private double pathCost(Pose2d startingPose, List<Pose2d> edgeWalk) {
    if (target == null) {
      return Double.MAX_VALUE;
    }

    if (edgeWalk.isEmpty()) {
      return distanceBetween(startingPose, target);
    }

    double total = distanceBetween(startingPose, edgeWalk.get(0));
    for (int i = 1; i < edgeWalk.size(); i++) {
      total += distanceBetween(edgeWalk.get(i - 1), edgeWalk.get(i));
    }
    total += distanceBetween(edgeWalk.get(edgeWalk.size() - 1), target);
    return total;
  }

  private void rebuildPlannedPath(Pose2d startingPose) {
    plannedPath.clear();
    if (target == null) {
      return;
    }

    // If the straight line is clear, skip the reef edges entirely.
    if (!segmentHitsReef(startingPose, target)) {
      plannedPath.add(target);
      return;
    }

    edges entry = findClosestEdge(startingPose);
    edges exit = findClosestEdgeByAngle(target);

    // Pick the shorter perimeter walk (clockwise vs counter-clockwise) so we skirt the reef.
    List<Pose2d> clockwiseWalk = buildEdgeWalk(entry, exit, true);
    List<Pose2d> counterClockwiseWalk = buildEdgeWalk(entry, exit, false);

    double clockwiseCost = pathCost(startingPose, clockwiseWalk);
    double counterClockwiseCost = pathCost(startingPose, counterClockwiseWalk);

    List<Pose2d> bestWalk =
        clockwiseCost <= counterClockwiseCost ? clockwiseWalk : counterClockwiseWalk;

    // Add corners until we have clear line-of-sight to the target, but stop if we start moving away
    // from the target side of the reef.
    Pose2d cursor = startingPose;
    double targetAngle = angleFromReefCenter(target);
    double prevAngleDiff = smallestAngleDiff(angleFromReefCenter(cursor), targetAngle);
    boolean addedCorner = false;
    for (Pose2d corner : bestWalk) {
      // If this leg would pass over the target, stop and go straight there.
      double targetCrossDist =
          distancePointToSegment(
              target.getTranslation(), cursor.getTranslation(), corner.getTranslation());
      if (targetCrossDist < TARGET_PASS_THROUGH_MARGIN) {
        break;
      }

      double cornerAngleDiff = smallestAngleDiff(angleFromReefCenter(corner), targetAngle);
      if (addedCorner && cornerAngleDiff > prevAngleDiff + ANGLE_DIVERGENCE_EPSILON) {
        break; // We started moving away from the target side; don't wrap past it.
      }

      plannedPath.add(corner);
      addedCorner = true;
      cursor = corner;
      prevAngleDiff = cornerAngleDiff;

      if (cornerAngleDiff <= ANGLE_MATCH_EPSILON) {
        break; // We are lined up with the target; stop wrapping.
      }
      if (!segmentHitsReef(cursor, target)) {
        break;
      }
    }
    if (plannedPath.isEmpty()
        || distanceBetween(plannedPath.get(plannedPath.size() - 1), target) > 1e-6) {
      plannedPath.add(target);
    }
  }

  private double computeIntermediateHandoffDistance(int index) {
    if (index >= plannedPath.size() - 1) {
      return 0.0;
    }

    double legDistance = distanceBetween(plannedPath.get(index), plannedPath.get(index + 1));
    double handoff =
        Math.max(MIN_HANDOFF_DISTANCE, Math.min(MAX_HANDOFF_DISTANCE, legDistance * 0.25));
    return handoff;
  }

  private void startAlignmentForCurrentIndex() {
    if (currentIndex >= plannedPath.size()) {
      nextTarget = () -> target;
      autoAlign = null;
      currentLegHandoffDistance = 0.0;
      return;
    }

    Pose2d goal = plannedPath.get(currentIndex);
    lastEdge = goal;
    nextTarget = () -> goal;

    // Configure alignment; intermediate legs hand off early with a nonzero end velocity
    boolean isFinalLeg = currentIndex == plannedPath.size() - 1;
    if (isFinalLeg) {
      currentLegHandoffDistance = 0.0;
      autoAlign = new AutoAlignToPoseCommand(drive, goal);
    } else {
      currentLegHandoffDistance = computeIntermediateHandoffDistance(currentIndex);
      autoAlign =
          new AutoAlignToPoseCommand(
              drive,
              goal,
              1.0,
              AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED,
              AlignConstants.ALIGN_TRANSLATION_TOLERANCE_METERS);
    }
    autoAlign.initialize();
  }

  @Override
  public void initialize() {
    if (target == null) {
      return;
    }

    rebuildPlannedPath(drive.getPose());
    currentIndex = 0;
    startAlignmentForCurrentIndex();
  }

  @Override
  public void execute() {
    if (target == null || autoAlign == null) return;

    Logger.recordOutput("PathWeave/currentPose", drive.getPose());
    if (nextTarget != null) {
      Logger.recordOutput("PathWeave/targetPose", nextTarget.get());
    }

    autoAlign.execute();

    Pose2d currentGoal =
        nextTarget != null && currentIndex < plannedPath.size() ? nextTarget.get() : target;
    boolean earlyHandoff =
        currentLegHandoffDistance > 0.0
            && drive.getPose().getTranslation().getDistance(currentGoal.getTranslation())
                <= currentLegHandoffDistance;
    boolean moreLegsRemain = currentIndex + 1 < plannedPath.size();

    if (earlyHandoff || autoAlign.isFinished()) {
      // Only stop the drive when we're on the final leg; keep motion continuous between hops.
      if (!moreLegsRemain) {
        autoAlign.end(false);
      }
      currentIndex++;
      startAlignmentForCurrentIndex();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (this.autoAlign != null) autoAlign.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return target == null || (autoAlign == null && currentIndex >= plannedPath.size());
  }
}
