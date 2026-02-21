package org.Griffins1884.frc2026.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class GyroIOPrimaryFallbackTest {
  private static final double EPSILON = 1e-6;

  @Test
  void usesPrimaryWhenConnected() {
    FakeGyro primary = new FakeGyro(true, 30.0, 1.0);
    FakeGyro secondary = new FakeGyro(true, -15.0, 2.0);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);
    GyroIO.GyroIOInputs out = new GyroIO.GyroIOInputs();

    fallback.updateInputs(out);

    assertTrue(out.connected);
    assertTrue(out.primaryConnected);
    assertTrue(out.secondaryConnected);
    assertFalse(out.usingSecondary);
    assertEquals(30.0, out.yawPosition.getDegrees(), EPSILON);
    assertEquals(1.0, out.yawVelocityRadPerSec, EPSILON);
  }

  @Test
  void killsPrimaryAndFallsBackToSecondaryWithContinuousYaw() {
    FakeGyro primary = new FakeGyro(true, 50.0, 0.1);
    FakeGyro secondary = new FakeGyro(true, 10.0, 0.2);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);
    GyroIO.GyroIOInputs out = new GyroIO.GyroIOInputs();

    fallback.updateInputs(out);
    assertEquals(50.0, out.yawPosition.getDegrees(), EPSILON);
    assertFalse(out.usingSecondary);

    // Simulate Pigeon drop-out. Debounce should require 3 bad cycles.
    primary.setState(false, 50.0, 0.1);
    secondary.setState(true, 12.0, 0.2);
    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);
    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);
    fallback.updateInputs(out);

    assertTrue(out.usingSecondary);
    assertTrue(out.connected);
    assertEquals(50.0, out.yawPosition.getDegrees(), EPSILON);
    assertEquals(50.0, out.odometryYawPositions[0].getDegrees(), EPSILON);

    // Secondary advances and should remain continuous with preserved offset.
    secondary.setState(true, 20.0, 0.2);
    fallback.updateInputs(out);
    assertEquals(58.0, out.yawPosition.getDegrees(), EPSILON);
    assertEquals(58.0, out.odometryYawPositions[0].getDegrees(), EPSILON);
  }

  @Test
  void requiresStablePrimaryBeforeSwitchingBack() {
    FakeGyro primary = new FakeGyro(false, 100.0, 0.1);
    FakeGyro secondary = new FakeGyro(true, 40.0, 0.2);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);
    GyroIO.GyroIOInputs out = new GyroIO.GyroIOInputs();

    fallback.updateInputs(out);
    fallback.updateInputs(out);
    fallback.updateInputs(out);
    assertTrue(out.usingSecondary);

    primary.setState(true, 105.0, 0.1);
    secondary.setState(true, 45.0, 0.2);
    for (int i = 0; i < 9; i++) {
      fallback.updateInputs(out);
      assertTrue(out.usingSecondary);
    }

    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);
    assertEquals(105.0, out.yawPosition.getDegrees(), EPSILON);
  }

  @Test
  void transientPrimaryBlipDoesNotTriggerFallback() {
    FakeGyro primary = new FakeGyro(true, 20.0, 0.1);
    FakeGyro secondary = new FakeGyro(true, -30.0, 0.2);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);
    GyroIO.GyroIOInputs out = new GyroIO.GyroIOInputs();

    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);

    primary.setState(false, 20.0, 0.1);
    secondary.setState(true, -28.0, 0.2);
    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);

    primary.setState(true, 21.0, 0.1);
    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);
    assertEquals(21.0, out.yawPosition.getDegrees(), EPSILON);
  }

  @Test
  void switchBackToPrimaryIsContinuousThenDecaysOffset() {
    FakeGyro primary = new FakeGyro(true, 0.0, 0.1);
    FakeGyro secondary = new FakeGyro(true, 90.0, 0.2);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);
    GyroIO.GyroIOInputs out = new GyroIO.GyroIOInputs();

    fallback.updateInputs(out);
    assertEquals(0.0, out.yawPosition.getDegrees(), EPSILON);

    primary.setState(false, 0.0, 0.1);
    secondary.setState(true, 90.0, 0.2);
    fallback.updateInputs(out);
    fallback.updateInputs(out);
    fallback.updateInputs(out);
    assertTrue(out.usingSecondary);
    assertEquals(0.0, out.yawPosition.getDegrees(), EPSILON);

    secondary.setState(true, 100.0, 0.2);
    fallback.updateInputs(out);
    assertEquals(10.0, out.yawPosition.getDegrees(), EPSILON);

    primary.setState(true, 30.0, 0.1);
    for (int i = 0; i < 9; i++) {
      fallback.updateInputs(out);
      assertTrue(out.usingSecondary);
    }

    fallback.updateInputs(out);
    assertFalse(out.usingSecondary);
    assertEquals(10.0, out.yawPosition.getDegrees(), EPSILON);

    fallback.updateInputs(out);
    assertEquals(11.0, out.yawPosition.getDegrees(), EPSILON);
  }

  @Test
  void resetYawForwardsToBothSensors() {
    FakeGyro primary = new FakeGyro(true, 0.0, 0.0);
    FakeGyro secondary = new FakeGyro(true, 0.0, 0.0);
    GyroIOPrimaryFallback fallback = new GyroIOPrimaryFallback(primary, secondary);

    fallback.resetYaw(90.0);

    assertEquals(1, primary.resetCount);
    assertEquals(1, secondary.resetCount);
    assertEquals(90.0, primary.lastResetYawDegrees, EPSILON);
    assertEquals(90.0, secondary.lastResetYawDegrees, EPSILON);
  }

  private static class FakeGyro implements GyroIO {
    private boolean connected;
    private double yawDegrees;
    private double yawRateRadPerSec;
    private double timestampSec = 1.0;
    private int resetCount = 0;
    private double lastResetYawDegrees = 0.0;

    private FakeGyro(boolean connected, double yawDegrees, double yawRateRadPerSec) {
      this.connected = connected;
      this.yawDegrees = yawDegrees;
      this.yawRateRadPerSec = yawRateRadPerSec;
    }

    private void setState(boolean connected, double yawDegrees, double yawRateRadPerSec) {
      this.connected = connected;
      this.yawDegrees = yawDegrees;
      this.yawRateRadPerSec = yawRateRadPerSec;
      timestampSec += 0.02;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = connected;
      inputs.yawPosition = Rotation2d.fromDegrees(yawDegrees);
      inputs.yawVelocityRadPerSec = yawRateRadPerSec;
      inputs.odometryYawTimestamps = new double[] {timestampSec};
      inputs.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(yawDegrees)};
    }

    @Override
    public void resetYaw(double yawDegrees) {
      resetCount++;
      lastResetYawDegrees = yawDegrees;
      this.yawDegrees = yawDegrees;
    }
  }
}
