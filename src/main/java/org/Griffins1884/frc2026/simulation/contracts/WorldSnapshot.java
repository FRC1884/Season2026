package org.Griffins1884.frc2026.simulation.contracts;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;

/** Authoritative world snapshot emitted by deterministic replay surfaces. */
public record WorldSnapshot(
    long simTimeNanos,
    int stepId,
    boolean predictionAvailable,
    boolean predictionFeasible,
    Pose3d releasePose,
    Pose3d impactPose,
    List<Pose3d> activeProjectilePoses,
    List<RigidBodyState> rigidBodyStates,
    List<ContactPointState> contactPointStates,
    List<WheelContactState> wheelContactStates) {
  public WorldSnapshot {
    if (simTimeNanos < 0L) {
      throw new IllegalArgumentException("simTimeNanos must be non-negative");
    }
    if (stepId < 0) {
      throw new IllegalArgumentException("stepId must be non-negative");
    }
    activeProjectilePoses =
        activeProjectilePoses != null ? List.copyOf(activeProjectilePoses) : List.of();
    rigidBodyStates = rigidBodyStates != null ? List.copyOf(rigidBodyStates) : List.of();
    contactPointStates = contactPointStates != null ? List.copyOf(contactPointStates) : List.of();
    wheelContactStates = wheelContactStates != null ? List.copyOf(wheelContactStates) : List.of();
  }

  public WorldSnapshot(
      long simTimeNanos,
      int stepId,
      boolean predictionAvailable,
      boolean predictionFeasible,
      Pose3d releasePose,
      Pose3d impactPose,
      List<Pose3d> activeProjectilePoses) {
    this(
        simTimeNanos,
        stepId,
        predictionAvailable,
        predictionFeasible,
        releasePose,
        impactPose,
        activeProjectilePoses,
        List.of(),
        List.of(),
        List.of());
  }
}
