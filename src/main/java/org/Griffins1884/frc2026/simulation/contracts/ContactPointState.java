package org.Griffins1884.frc2026.simulation.contracts;

/** Serializable contact state including resolved impulses. */
public record ContactPointState(
    int bodyAId,
    int bodyBId,
    double pointX,
    double pointY,
    double pointZ,
    double normalX,
    double normalY,
    double normalZ,
    double penetrationDepth,
    double normalImpulse,
    double tangentImpulse1,
    double tangentImpulse2) {}
