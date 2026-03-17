package org.Griffins1884.frc2026.mechanisms;

import java.util.List;
import java.util.Set;

/**
 * Normalized telemetry snapshot for all migrated mechanisms.
 *
 * <p>This is intentionally broader than the current family-specific IO inputs so the later
 * dashboard/config work can render one consistent tree.
 */
public record MechanismTelemetry(
    String mechanismKey,
    String displayName,
    MechanismHealth health,
    boolean sensorConnected,
    boolean forwardLimitSwitch,
    boolean reverseLimitSwitch,
    double position,
    double velocity,
    double appliedOutput,
    double appliedVoltage,
    double supplyCurrentAmps,
    double torqueCurrentAmps,
    double temperatureCelsius,
    double targetPosition,
    double targetVelocity,
    double closedLoopError,
    String controlMode,
    List<MotorTelemetry> motors,
    Set<String> warnings,
    Set<String> faults,
    String configSnapshot) {

  /** Telemetry categories used by logging and NetworkTables publish policies. */
  public enum Signal {
    IDENTITY,
    CONNECTION,
    FAULTS,
    TEMPERATURE,
    VOLTAGE,
    CURRENT,
    POSITION,
    VELOCITY,
    APPLIED_OUTPUT,
    TARGET,
    CLOSED_LOOP,
    LIMIT_SWITCH,
    SENSOR_STATE,
    HEALTH,
    CONFIG_SNAPSHOT
  }

  /** Per-motor device snapshot for multi-motor mechanisms. */
  public record MotorTelemetry(
      int canId,
      boolean connected,
      double appliedOutput,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double temperatureCelsius) {}
}
