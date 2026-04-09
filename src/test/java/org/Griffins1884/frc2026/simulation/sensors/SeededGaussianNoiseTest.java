package org.Griffins1884.frc2026.simulation.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SeededGaussianNoiseTest {
  @Test
  void sameSeedProducesSameNoiseSequence() {
    SeededGaussianNoise first = new SeededGaussianNoise(42L, 0.5);
    SeededGaussianNoise second = new SeededGaussianNoise(42L, 0.5);
    for (int i = 0; i < 8; i++) {
      assertEquals(first.sample(), second.sample(), 1e-12);
    }
  }
}
