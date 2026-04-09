package org.Griffins1884.frc2026.simulation.engine;

/** Surface material properties for collision and wheel interactions. */
public record PhysicsMaterial(
    double frictionCoefficient,
    double restitutionCoefficient,
    double rollingResistanceCoefficient,
    double lateralFrictionCoefficient) {
  public static final PhysicsMaterial DEFAULT = new PhysicsMaterial(0.8, 0.05, 0.015, 0.9);
}
