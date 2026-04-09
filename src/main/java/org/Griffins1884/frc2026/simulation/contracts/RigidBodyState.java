package org.Griffins1884.frc2026.simulation.contracts;

/** Serializable rigid body state for deterministic physics replays. */
public record RigidBodyState(
    int id,
    String name,
    String shapeType,
    boolean dynamic,
    double positionX,
    double positionY,
    double positionZ,
    double quaternionW,
    double quaternionX,
    double quaternionY,
    double quaternionZ,
    double linearVelocityX,
    double linearVelocityY,
    double linearVelocityZ,
    double angularVelocityX,
    double angularVelocityY,
    double angularVelocityZ) {}
