package org.Griffins1884.frc2026.simulation.engine;

import edu.wpi.first.math.geometry.Translation3d;

/** Fixed target geometry for the local deterministic engine. */
public record ShotEngineConfig(
    Translation3d targetPositionMeters,
    Translation3d targetConeTopMeters,
    double openingRadiusMeters,
    double topOpeningRadiusMeters,
    double coneClearanceMeters) {
  public static ShotEngineConfig defaultConfig() {
    return new ShotEngineConfig(
        new Translation3d(4.2, 1.95, 2.05), new Translation3d(4.2, 1.95, 2.45), 0.35, 0.45, 0.2);
  }
}
