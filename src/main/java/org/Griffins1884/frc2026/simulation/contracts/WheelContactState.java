package org.Griffins1884.frc2026.simulation.contracts;

/** Serializable wheel-ground contact telemetry for deterministic replay. */
public record WheelContactState(
    int wheelIndex,
    boolean contacting,
    double pointX,
    double pointY,
    double pointZ,
    double normalX,
    double normalY,
    double normalZ,
    double normalForceNewtons,
    double slipRatio,
    double slipAngleRadians,
    double longitudinalForceX,
    double longitudinalForceY,
    double longitudinalForceZ,
    double lateralForceX,
    double lateralForceY,
    double lateralForceZ) {}
