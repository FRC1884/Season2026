package org.Griffins1884.frc2026.simulation.engine;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Telemetry emitted for each wheel-ground contact patch. */
public record WheelContactTelemetry(
    int wheelIndex,
    boolean contacting,
    Vec3 point,
    Vec3 normal,
    double normalForceNewtons,
    double slipRatio,
    double slipAngleRadians,
    Vec3 longitudinalForce,
    Vec3 lateralForce) {}
