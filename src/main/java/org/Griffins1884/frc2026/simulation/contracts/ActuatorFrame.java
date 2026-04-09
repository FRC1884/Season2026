package org.Griffins1884.frc2026.simulation.contracts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Canonical actuator command frame for deterministic simulation replay. */
public record ActuatorFrame(
    long simTimeNanos,
    int stepId,
    Pose2d robotPose,
    Translation2d fieldVelocityMetersPerSecond,
    double turretYawRadians,
    double pivotMotorRotations,
    double shooterRpm,
    boolean shooterArmed) {
  public ActuatorFrame {
    if (simTimeNanos < 0L) {
      throw new IllegalArgumentException("simTimeNanos must be non-negative");
    }
    if (stepId < 0) {
      throw new IllegalArgumentException("stepId must be non-negative");
    }
    robotPose = robotPose != null ? robotPose : new Pose2d();
    fieldVelocityMetersPerSecond =
        fieldVelocityMetersPerSecond != null ? fieldVelocityMetersPerSecond : new Translation2d();
  }
}
