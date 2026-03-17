package org.Griffins1884.frc2026.mechanisms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/** Declarative definition for a non-drivetrain, non-vision mechanism. */
public record MechanismDefinition(
    String key,
    String displayName,
    MechanismType mechanismType,
    MechanismConfig config,
    TelemetryConfig telemetry,
    FaultPolicy faultPolicy,
    SimulationConfig simulation) {

  public MechanismDefinition {
    key = requireText(key, "key");
    displayName = requireText(displayName, "displayName");
    mechanismType = Objects.requireNonNull(mechanismType, "mechanismType");
    config = Objects.requireNonNull(config, "config").validated();
    telemetry = telemetry != null ? telemetry.normalized() : TelemetryConfig.defaults();
    faultPolicy = faultPolicy != null ? faultPolicy : FaultPolicy.WARN_ONLY;
    simulation = simulation != null ? simulation.normalized() : SimulationConfig.disabled();
  }

  public static Builder builder(String key, String displayName, MechanismType mechanismType) {
    return new Builder(key, displayName, mechanismType);
  }

  private static String requireText(String value, String fieldName) {
    if (value == null || value.isBlank()) {
      throw new IllegalArgumentException(fieldName + " must be non-blank");
    }
    return value;
  }

  public enum MechanismType {
    ROLLER,
    FLYWHEEL,
    INTAKE,
    INDEXER,
    PIVOT,
    TURRET,
    ELEVATOR,
    SPINDEXER,
    CUSTOM
  }

  public enum MotorControllerType {
    TALON_FX,
    SPARK_MAX,
    SPARK_FLEX,
    SIMULATION_ONLY,
    OTHER;

    public boolean supportsKrakenExtensions() {
      return this == TALON_FX;
    }
  }

  public enum FeedbackSensorType {
    INTERNAL,
    DUTY_CYCLE_ABSOLUTE,
    CANCODER,
    ANALOG_ABSOLUTE,
    REMOTE,
    SIMULATION_ONLY,
    NONE
  }

  public enum NeutralMode {
    BRAKE,
    COAST
  }

  public enum FollowerMode {
    INDEPENDENT,
    FOLLOW_LEADER,
    FOLLOW_LEADER_INVERTED
  }

  public enum FaultPolicy {
    WARN_ONLY,
    DISABLE_OUTPUT,
    LATCHED_STOP
  }

  public enum Capability {
    OPEN_LOOP,
    CLOSED_LOOP,
    CHARACTERIZATION
  }

  public record PIDConfig(double kP, double kI, double kD) {}

  public record FeedforwardConfig(double kS, double kG, double kV, double kA) {}

  public record MotionProfileConfig(double maxVelocity, double maxAcceleration, double jerk) {
    public MotionProfileConfig {
      if (maxVelocity < 0.0) {
        throw new IllegalArgumentException("maxVelocity must be >= 0");
      }
      if (maxAcceleration < 0.0) {
        throw new IllegalArgumentException("maxAcceleration must be >= 0");
      }
      if (jerk < 0.0) {
        throw new IllegalArgumentException("jerk must be >= 0");
      }
    }
  }

  public record SoftLimitConfig(boolean enabled, double min, double max) {
    public SoftLimitConfig {
      if (enabled && max < min) {
        throw new IllegalArgumentException("soft limit max must be >= min");
      }
    }
  }

  public record KrakenFeatureConfig(
      boolean enabled,
      boolean useFoc,
      boolean useTorqueCurrentVelocity,
      int statusSignalHz,
      boolean extendedFaults) {
    public KrakenFeatureConfig {
      if (statusSignalHz < 0) {
        throw new IllegalArgumentException("statusSignalHz must be >= 0");
      }
    }

    public static KrakenFeatureConfig disabled() {
      return new KrakenFeatureConfig(false, false, false, 0, false);
    }
  }

  public record MotorConfig(
      int canId,
      boolean inverted,
      FollowerMode followerMode,
      int supplyCurrentLimitAmps,
      int statorCurrentLimitAmps,
      double peakForwardOutput,
      double peakReverseOutput) {
    public MotorConfig {
      if (canId < 0) {
        throw new IllegalArgumentException("CAN ID must be >= 0");
      }
      followerMode = followerMode != null ? followerMode : FollowerMode.INDEPENDENT;
      if (supplyCurrentLimitAmps < 0 || statorCurrentLimitAmps < 0) {
        throw new IllegalArgumentException("current limits must be >= 0");
      }
      if (peakForwardOutput < 0.0 || peakForwardOutput > 1.0) {
        throw new IllegalArgumentException("peakForwardOutput must be within [0, 1]");
      }
      if (peakReverseOutput > 0.0 || peakReverseOutput < -1.0) {
        throw new IllegalArgumentException("peakReverseOutput must be within [-1, 0]");
      }
    }
  }

  public record MotorGroupConfig(
      MotorControllerType controllerType,
      String canBus,
      List<MotorConfig> motors,
      int leaderIndex,
      double gearRatio,
      FeedbackSensorType feedbackSensorType,
      NeutralMode neutralMode,
      boolean voltageCompensationEnabled,
      double voltageCompSaturationVolts,
      double openLoopRampSeconds,
      double closedLoopRampSeconds,
      SoftLimitConfig softLimits) {
    public MotorGroupConfig {
      controllerType = Objects.requireNonNull(controllerType, "controllerType");
      canBus = canBus != null ? canBus : "";
      motors =
          Collections.unmodifiableList(new ArrayList<>(Objects.requireNonNull(motors, "motors")));
      if (motors.isEmpty()) {
        throw new IllegalArgumentException("motors must not be empty");
      }
      if (leaderIndex < 0 || leaderIndex >= motors.size()) {
        throw new IllegalArgumentException("leaderIndex is out of range");
      }
      if (gearRatio <= 0.0) {
        throw new IllegalArgumentException("gearRatio must be > 0");
      }
      feedbackSensorType =
          feedbackSensorType != null ? feedbackSensorType : FeedbackSensorType.INTERNAL;
      neutralMode = neutralMode != null ? neutralMode : NeutralMode.BRAKE;
      if (voltageCompensationEnabled && voltageCompSaturationVolts <= 0.0) {
        throw new IllegalArgumentException("voltageCompSaturationVolts must be > 0 when enabled");
      }
      if (openLoopRampSeconds < 0.0 || closedLoopRampSeconds < 0.0) {
        throw new IllegalArgumentException("ramp seconds must be >= 0");
      }
      softLimits = softLimits != null ? softLimits : new SoftLimitConfig(false, 0.0, 0.0);
      validateUniqueCanIds(motors);
    }

    private static void validateUniqueCanIds(List<MotorConfig> motors) {
      Set<Integer> ids = new HashSet<>();
      for (MotorConfig motor : motors) {
        if (!ids.add(motor.canId())) {
          throw new IllegalArgumentException("duplicate CAN ID in motor group: " + motor.canId());
        }
      }
    }
  }

  public record ClosedLoopConfig(
      PIDConfig pid,
      FeedforwardConfig feedforward,
      MotionProfileConfig motionProfile,
      double positionTolerance,
      double velocityTolerance,
      double maxVoltage) {
    public ClosedLoopConfig {
      pid = Objects.requireNonNull(pid, "pid");
      feedforward = feedforward != null ? feedforward : new FeedforwardConfig(0.0, 0.0, 0.0, 0.0);
      if (positionTolerance < 0.0 || velocityTolerance < 0.0) {
        throw new IllegalArgumentException("tolerances must be >= 0");
      }
      if (maxVoltage <= 0.0) {
        throw new IllegalArgumentException("maxVoltage must be > 0");
      }
    }
  }

  public record MechanismConfig(
      MotorGroupConfig motorGroup,
      ClosedLoopConfig closedLoop,
      KrakenFeatureConfig krakenFeatures,
      Set<Capability> capabilities) {
    public MechanismConfig {
      motorGroup = Objects.requireNonNull(motorGroup, "motorGroup");
      closedLoop = Objects.requireNonNull(closedLoop, "closedLoop");
      krakenFeatures = krakenFeatures != null ? krakenFeatures : KrakenFeatureConfig.disabled();
      capabilities = normalizeCapabilities(capabilities);
    }

    public MechanismConfig validated() {
      if (krakenFeatures.enabled() && !motorGroup.controllerType().supportsKrakenExtensions()) {
        throw new IllegalArgumentException(
            "Kraken features are only valid for TalonFX-based mechanisms");
      }
      if (!supportsOpenLoop() && !supportsClosedLoop()) {
        throw new IllegalArgumentException(
            "mechanism must support at least one runtime control mode");
      }
      return this;
    }

    public boolean supportsOpenLoop() {
      return capabilities.contains(Capability.OPEN_LOOP);
    }

    public boolean supportsClosedLoop() {
      return capabilities.contains(Capability.CLOSED_LOOP);
    }

    public boolean supportsCharacterization() {
      return capabilities.contains(Capability.CHARACTERIZATION);
    }

    public static Set<Capability> capabilities(Capability... capabilities) {
      if (capabilities == null || capabilities.length == 0) {
        return Collections.emptySet();
      }
      EnumSet<Capability> normalized = EnumSet.noneOf(Capability.class);
      Collections.addAll(normalized, capabilities);
      return Collections.unmodifiableSet(normalized);
    }

    private static Set<Capability> normalizeCapabilities(Set<Capability> capabilities) {
      if (capabilities == null || capabilities.isEmpty()) {
        return Collections.emptySet();
      }
      return Collections.unmodifiableSet(EnumSet.copyOf(capabilities));
    }
  }

  public record TelemetryConfig(
      Set<MechanismTelemetry.Signal> loggedSignals,
      Set<MechanismTelemetry.Signal> publishedSignals,
      boolean includeConfigSnapshot,
      boolean includeHealthSummary) {
    public TelemetryConfig normalized() {
      return new TelemetryConfig(
          normalizeSignals(loggedSignals),
          normalizeSignals(publishedSignals),
          includeConfigSnapshot,
          includeHealthSummary);
    }

    public static TelemetryConfig defaults() {
      EnumSet<MechanismTelemetry.Signal> defaultSignals =
          EnumSet.of(
              MechanismTelemetry.Signal.IDENTITY,
              MechanismTelemetry.Signal.CONNECTION,
              MechanismTelemetry.Signal.FAULTS,
              MechanismTelemetry.Signal.HEALTH,
              MechanismTelemetry.Signal.TARGET);
      return new TelemetryConfig(defaultSignals, defaultSignals, true, true);
    }

    private static Set<MechanismTelemetry.Signal> normalizeSignals(
        Set<MechanismTelemetry.Signal> signals) {
      if (signals == null || signals.isEmpty()) {
        return Collections.emptySet();
      }
      return Collections.unmodifiableSet(EnumSet.copyOf(signals));
    }
  }

  public record SimulationConfig(
      boolean enabled,
      double reduction,
      double momentOfInertia,
      boolean gravityAware,
      boolean publishSimState) {
    public SimulationConfig {
      if (enabled && reduction <= 0.0) {
        throw new IllegalArgumentException("simulation reduction must be > 0 when enabled");
      }
      if (enabled && momentOfInertia < 0.0) {
        throw new IllegalArgumentException("simulation momentOfInertia must be >= 0 when enabled");
      }
    }

    public SimulationConfig normalized() {
      return this;
    }

    public static SimulationConfig disabled() {
      return new SimulationConfig(false, 1.0, 0.0, false, false);
    }
  }

  /** Small fluent builder so new mechanisms can be declared without giant constructors. */
  public static final class Builder {
    private final String key;
    private final String displayName;
    private final MechanismType mechanismType;
    private MechanismConfig config;
    private TelemetryConfig telemetry;
    private FaultPolicy faultPolicy;
    private SimulationConfig simulation;

    private Builder(String key, String displayName, MechanismType mechanismType) {
      this.key = key;
      this.displayName = displayName;
      this.mechanismType = mechanismType;
    }

    public Builder config(MechanismConfig config) {
      this.config = config;
      return this;
    }

    public Builder telemetry(TelemetryConfig telemetry) {
      this.telemetry = telemetry;
      return this;
    }

    public Builder faultPolicy(FaultPolicy faultPolicy) {
      this.faultPolicy = faultPolicy;
      return this;
    }

    public Builder simulation(SimulationConfig simulation) {
      this.simulation = simulation;
      return this;
    }

    public MechanismDefinition build() {
      return new MechanismDefinition(
          key, displayName, mechanismType, config, telemetry, faultPolicy, simulation);
    }
  }
}
