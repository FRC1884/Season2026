package org.Griffins1884.frc2026.simulation.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/** Immutable staged sensor output for deterministic active SIM reads. */
public record SwerveSensorSample(
    double timestampSeconds,
    Rotation2d yaw,
    Rotation2d pitch,
    Rotation2d roll,
    double yawVelocityRadPerSec,
    double pitchVelocityRadPerSec,
    double rollVelocityRadPerSec,
    double[] cachedTimestamps,
    Rotation2d[] cachedYawPositions,
    double[][] drivePositionsRad,
    double[][] driveVelocitiesRadPerSec,
    Rotation2d[][] turnPositions,
    double[][] turnVelocitiesRadPerSec,
    double[][] turnPositionsRotations) {}
