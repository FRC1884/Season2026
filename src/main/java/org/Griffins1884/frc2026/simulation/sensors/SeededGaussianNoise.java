package org.Griffins1884.frc2026.simulation.sensors;

import java.util.Random;

/** Deterministic Gaussian noise source seeded once per simulated sensor stream. */
public final class SeededGaussianNoise {
  private final Random random;
  private final double standardDeviation;

  public SeededGaussianNoise(long seed, double standardDeviation) {
    random = new Random(seed);
    this.standardDeviation = standardDeviation;
  }

  public double sample() {
    return random.nextGaussian() * standardDeviation;
  }
}
