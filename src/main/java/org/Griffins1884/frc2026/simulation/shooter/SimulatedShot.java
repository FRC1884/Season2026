package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.Griffins1884.frc2026.util.ballistics.ShotModel;

/** Solved field-space shot data used for prediction and projectile spawning. */
public record SimulatedShot(
    boolean feasible,
    Pose3d releasePose,
    Translation3d initialVelocityMetersPerSecond,
    Pose3d[] predictedSamplePoses,
    Pose3d predictedImpactPose,
    ShotModel.ShotSolution solution) {}
