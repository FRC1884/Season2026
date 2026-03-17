package org.Griffins1884.frc2026.mechanisms;

import com.ctre.phoenix6.CANBus;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerConstants;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakeConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotConstants;
import org.Griffins1884.frc2026.subsystems.intake.ToothRolloutConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotConstants;
import org.Griffins1884.frc2026.subsystems.turret.TurretConstants;

/**
 * Repo-specific mechanism catalog for the current non-drivetrain, non-vision systems.
 *
 * <p>These definitions are the migration target for the old generic family classes and the config
 * source for future dashboard/debug tooling.
 */
public final class RobotMechanismDefinitions {
  private static final Set<MechanismTelemetry.Signal> fullTelemetry =
      EnumSet.allOf(MechanismTelemetry.Signal.class);
  private static final Set<MechanismDefinition.Capability> standardCapabilities =
      MechanismDefinition.MechanismConfig.capabilities(
          MechanismDefinition.Capability.OPEN_LOOP,
          MechanismDefinition.Capability.CLOSED_LOOP,
          MechanismDefinition.Capability.CHARACTERIZATION);

  public static final MechanismDefinition INTAKE =
      MechanismDefinition.builder("intake", "Intake", MechanismDefinition.MechanismType.INTAKE)
          .config(
              mechanismConfig(
                  motorGroup(
                      MechanismDefinition.MotorControllerType.TALON_FX,
                      canBusName(IntakeConstants.CAN_BUS),
                      List.of(
                          motor(
                              IntakeConstants.INTAKE_IDS[0],
                              IntakeConstants.INTAKE_INVERTED[0],
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              IntakeConstants.CURRENT_LIMIT_AMPS)),
                      IntakeConstants.REDUCTION,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(IntakeConstants.BRAKE_MODE),
                      IntakeConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimitsDisabled()),
                  closedLoop(
                      IntakeConstants.GAINS,
                      0.0,
                      IntakeConstants.VELOCITY_TOLERANCE,
                      IntakeConstants.MAX_VOLTAGE),
                  MechanismDefinition.KrakenFeatureConfig.disabled()))
          .telemetry(fullTelemetry())
          .simulation(simulation(IntakeConstants.REDUCTION, 0.0, false))
          .build();

  public static final MechanismDefinition INDEXER =
      MechanismDefinition.builder("indexer", "Indexer", MechanismDefinition.MechanismType.INDEXER)
          .config(
              mechanismConfig(
                  motorGroup(
                      MechanismDefinition.MotorControllerType.TALON_FX,
                      canBusName(IndexerConstants.CAN_BUS),
                      List.of(
                          motor(
                              IndexerConstants.INDEXER_IDS[0],
                              IndexerConstants.INDEXER_INVERTED[0],
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              IndexerConstants.CURRENT_LIMIT_AMPS)),
                      IndexerConstants.REDUCTION,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(IndexerConstants.BRAKE_MODE),
                      IndexerConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimitsDisabled()),
                  closedLoop(
                      IndexerConstants.GAINS,
                      0.0,
                      IndexerConstants.VELOCITY_TOLERANCE,
                      IndexerConstants.MAX_VOLTAGE),
                  MechanismDefinition.KrakenFeatureConfig.disabled()))
          .telemetry(fullTelemetry())
          .simulation(simulation(IndexerConstants.REDUCTION, 0.0, false))
          .build();

  public static final MechanismDefinition SHOOTER =
      MechanismDefinition.builder("shooter", "Shooter", MechanismDefinition.MechanismType.FLYWHEEL)
          .config(
              mechanismConfig(
                  motorGroup(
                      MechanismDefinition.MotorControllerType.TALON_FX,
                      canBusName(ShooterConstants.CAN_BUS),
                      List.of(
                          motor(
                              ShooterConstants.SHOOTER_IDS[0],
                              ShooterConstants.SHOOTER_INVERTED[0],
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              ShooterConstants.CURRENT_LIMIT_AMPS),
                          motor(
                              ShooterConstants.SHOOTER_IDS[1],
                              ShooterConstants.SHOOTER_INVERTED[1],
                              MechanismDefinition.FollowerMode.FOLLOW_LEADER_INVERTED,
                              ShooterConstants.CURRENT_LIMIT_AMPS)),
                      ShooterConstants.REDUCTION,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(ShooterConstants.BRAKE_MODE),
                      ShooterConstants.MAX_VOLTAGE,
                      0.0,
                      ShooterConstants.CLOSED_LOOP_RAMP_SECONDS,
                      softLimitsDisabled()),
                  closedLoop(
                      ShooterConstants.GAINS,
                      0.0,
                      ShooterConstants.VELOCITY_TOLERANCE,
                      ShooterConstants.MAX_VOLTAGE),
                  new MechanismDefinition.KrakenFeatureConfig(true, true, true, 100, true)))
          .telemetry(fullTelemetry())
          .simulation(simulation(ShooterConstants.REDUCTION, 0.0, false))
          .build();

  public static final MechanismDefinition INTAKE_PIVOT =
      MechanismDefinition.builder(
              "intakePivot", "IntakePivot", MechanismDefinition.MechanismType.PIVOT)
          .config(
              mechanismConfig(
                  motorGroup(
                      motorController(IntakePivotConstants.MOTOR_CONTROLLER),
                      canBusName(IntakePivotConstants.CAN_BUS),
                      List.of(
                          motor(
                              IntakePivotConstants.MOTOR_ID[0],
                              IntakePivotConstants.INVERTED[0],
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              IntakePivotConstants.CURRENT_LIMIT_AMPS),
                          motor(
                              IntakePivotConstants.MOTOR_ID[1],
                              IntakePivotConstants.INVERTED[1],
                              MechanismDefinition.FollowerMode.FOLLOW_LEADER_INVERTED,
                              IntakePivotConstants.CURRENT_LIMIT_AMPS)),
                      IntakePivotConstants.POSITION_COEFFICIENT,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(IntakePivotConstants.BRAKE_MODE),
                      IntakePivotConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimits(
                          IntakePivotConstants.SOFT_LIMITS_ENABLED,
                          IntakePivotConstants.SOFT_LIMIT_MIN,
                          IntakePivotConstants.SOFT_LIMIT_MAX)),
                  closedLoop(
                      IntakePivotConstants.GAINS,
                      IntakePivotConstants.POSITION_TOLERANCE,
                      0.0,
                      IntakePivotConstants.MAX_VOLTAGE,
                      new MechanismDefinition.MotionProfileConfig(
                          IntakePivotConstants.MOTION_MAGIC_CRUISE_VEL.get(),
                          IntakePivotConstants.MOTION_MAGIC_ACCEL.get(),
                          IntakePivotConstants.MOTION_MAGIC_JERK.get())),
                  krakenFeaturesFor(motorController(IntakePivotConstants.MOTOR_CONTROLLER))))
          .telemetry(fullTelemetry())
          .simulation(simulation(IntakePivotConstants.POSITION_COEFFICIENT, 0.0, true))
          .build();

  public static final MechanismDefinition SHOOTER_PIVOT =
      MechanismDefinition.builder(
              "shooterPivot", "ShooterPivot", MechanismDefinition.MechanismType.PIVOT)
          .config(
              mechanismConfig(
                  motorGroup(
                      motorController(ShooterPivotConstants.MOTOR_CONTROLLER),
                      canBusName(ShooterPivotConstants.CAN_BUS),
                      List.of(
                          motor(
                              ShooterPivotConstants.MOTOR_ID[0],
                              ShooterPivotConstants.INVERTED[0],
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              ShooterPivotConstants.CURRENT_LIMIT_AMPS)),
                      ShooterPivotConstants.POSITION_COEFFICIENT,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(ShooterPivotConstants.BRAKE_MODE),
                      ShooterPivotConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimits(
                          ShooterPivotConstants.SOFT_LIMITS_ENABLED,
                          ShooterPivotConstants.SOFT_LIMIT_MIN,
                          ShooterPivotConstants.SOFT_LIMIT_MAX)),
                  closedLoop(
                      ShooterPivotConstants.GAINS,
                      ShooterPivotConstants.POSITION_TOLERANCE,
                      0.0,
                      ShooterPivotConstants.MAX_VOLTAGE,
                      new MechanismDefinition.MotionProfileConfig(
                          ShooterPivotConstants.MOTION_MAGIC_CRUISE_VEL.get(),
                          ShooterPivotConstants.MOTION_MAGIC_ACCEL.get(),
                          ShooterPivotConstants.MOTION_MAGIC_JERK.get())),
                  krakenFeaturesFor(motorController(ShooterPivotConstants.MOTOR_CONTROLLER))))
          .telemetry(fullTelemetry())
          .simulation(simulation(ShooterPivotConstants.POSITION_COEFFICIENT, 0.0, true))
          .build();

  public static final MechanismDefinition TURRET =
      MechanismDefinition.builder("turret", "Turret", MechanismDefinition.MechanismType.TURRET)
          .config(
              mechanismConfig(
                  motorGroup(
                      motorController(TurretConstants.MOTOR_CONTROLLER),
                      "rio",
                      List.of(
                          motor(
                              TurretConstants.TURRET_ID,
                              TurretConstants.INVERTED,
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              TurretConstants.CURRENT_LIMIT_AMPS)),
                      TurretConstants.GEAR_RATIO,
                      TurretConstants.USE_ABSOLUTE_ENCODER
                          ? MechanismDefinition.FeedbackSensorType.DUTY_CYCLE_ABSOLUTE
                          : MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(TurretConstants.BRAKE_MODE),
                      TurretConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimits(
                          TurretConstants.SOFT_LIMITS_ENABLED,
                          TurretConstants.SOFT_LIMIT_MIN_RAD,
                          TurretConstants.SOFT_LIMIT_MAX_RAD)),
                  closedLoop(
                      new GlobalConstants.Gains(
                          "Turret/Definition",
                          TurretConstants.KP.get(),
                          TurretConstants.KI.get(),
                          TurretConstants.KD.get()),
                      TurretConstants.POSITION_TOLERANCE_RAD,
                      0.0,
                      TurretConstants.MAX_VOLTAGE,
                      new MechanismDefinition.MotionProfileConfig(
                          TurretConstants.MAX_VELOCITY_RAD_PER_SEC,
                          TurretConstants.MAX_ACCEL_RAD_PER_SEC2,
                          0.0)),
                  krakenFeaturesFor(motorController(TurretConstants.MOTOR_CONTROLLER))))
          .telemetry(fullTelemetry())
          .simulation(simulation(TurretConstants.GEAR_RATIO, TurretConstants.SIM_MOI, false))
          .build();

  public static final MechanismDefinition TOOTH_ROLLOUT =
      MechanismDefinition.builder(
              "toothRollout", "ToothRollout", MechanismDefinition.MechanismType.CUSTOM)
          .config(
              mechanismConfig(
                  motorGroup(
                      ToothRolloutConstants.MOTOR_CONTROLLER,
                      ToothRolloutConstants.CAN_BUS,
                      List.of(
                          motor(
                              ToothRolloutConstants.MOTOR_ID,
                              ToothRolloutConstants.INVERTED,
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              ToothRolloutConstants.CURRENT_LIMIT_AMPS)),
                      ToothRolloutConstants.REDUCTION,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(ToothRolloutConstants.BRAKE_MODE),
                      ToothRolloutConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimitsDisabled()),
                  closedLoop(
                      ToothRolloutConstants.GAINS, 0.0, 0.0, ToothRolloutConstants.MAX_VOLTAGE),
                  MechanismDefinition.KrakenFeatureConfig.disabled()))
          .telemetry(fullTelemetry())
          .simulation(simulation(ToothRolloutConstants.REDUCTION, 0.0, false))
          .build();

  public static final MechanismDefinition SPINDEXER =
      MechanismDefinition.builder(
              "spindexer", "Spindexer", MechanismDefinition.MechanismType.SPINDEXER)
          .config(
              mechanismConfig(
                  motorGroup(
                      SpindexerConstants.MOTOR_CONTROLLER,
                      SpindexerConstants.CAN_BUS,
                      List.of(
                          motor(
                              SpindexerConstants.MOTOR_ID,
                              SpindexerConstants.INVERTED,
                              MechanismDefinition.FollowerMode.INDEPENDENT,
                              SpindexerConstants.CURRENT_LIMIT_AMPS)),
                      SpindexerConstants.REDUCTION,
                      MechanismDefinition.FeedbackSensorType.INTERNAL,
                      neutralMode(SpindexerConstants.BRAKE_MODE),
                      SpindexerConstants.MAX_VOLTAGE,
                      0.0,
                      0.0,
                      softLimitsDisabled()),
                  closedLoop(
                      SpindexerConstants.GAINS,
                      0.0,
                      SpindexerConstants.VELOCITY_TOLERANCE,
                      SpindexerConstants.MAX_VOLTAGE),
                  MechanismDefinition.KrakenFeatureConfig.disabled()))
          .telemetry(fullTelemetry())
          .simulation(simulation(SpindexerConstants.REDUCTION, 0.0, false))
          .build();

  private static final List<MechanismDefinition> allDefinitions =
      List.of(
          INTAKE, INDEXER, SHOOTER, INTAKE_PIVOT, SHOOTER_PIVOT, TURRET, TOOTH_ROLLOUT, SPINDEXER);

  private RobotMechanismDefinitions() {}

  public static List<MechanismDefinition> all() {
    return allDefinitions;
  }

  public static MechanismDefinition forKey(String key) {
    for (MechanismDefinition definition : allDefinitions) {
      if (definition.key().equalsIgnoreCase(key)) {
        return definition;
      }
    }
    throw new IllegalArgumentException("Unknown mechanism key: " + key);
  }

  private static MechanismDefinition.TelemetryConfig fullTelemetry() {
    return new MechanismDefinition.TelemetryConfig(fullTelemetry, fullTelemetry, true, true);
  }

  private static MechanismDefinition.MechanismConfig mechanismConfig(
      MechanismDefinition.MotorGroupConfig motorGroup,
      MechanismDefinition.ClosedLoopConfig closedLoop,
      MechanismDefinition.KrakenFeatureConfig krakenFeatures) {
    return new MechanismDefinition.MechanismConfig(
        motorGroup, closedLoop, krakenFeatures, standardCapabilities);
  }

  private static MechanismDefinition.MotorConfig motor(
      int canId,
      boolean inverted,
      MechanismDefinition.FollowerMode followerMode,
      int currentLimitAmps) {
    return new MechanismDefinition.MotorConfig(
        canId, inverted, followerMode, currentLimitAmps, currentLimitAmps, 1.0, -1.0);
  }

  private static MechanismDefinition.MotorGroupConfig motorGroup(
      MechanismDefinition.MotorControllerType controllerType,
      String canBus,
      List<MechanismDefinition.MotorConfig> motors,
      double gearRatio,
      MechanismDefinition.FeedbackSensorType feedbackSensorType,
      MechanismDefinition.NeutralMode neutralMode,
      double voltageCompSaturationVolts,
      double openLoopRampSeconds,
      double closedLoopRampSeconds,
      MechanismDefinition.SoftLimitConfig softLimits) {
    return new MechanismDefinition.MotorGroupConfig(
        controllerType,
        canBus,
        motors,
        0,
        gearRatio,
        feedbackSensorType,
        neutralMode,
        true,
        voltageCompSaturationVolts,
        openLoopRampSeconds,
        closedLoopRampSeconds,
        softLimits);
  }

  private static MechanismDefinition.ClosedLoopConfig closedLoop(
      GlobalConstants.Gains gains,
      double positionTolerance,
      double velocityTolerance,
      double maxVoltage) {
    return closedLoop(gains, positionTolerance, velocityTolerance, maxVoltage, null);
  }

  private static MechanismDefinition.ClosedLoopConfig closedLoop(
      GlobalConstants.Gains gains,
      double positionTolerance,
      double velocityTolerance,
      double maxVoltage,
      MechanismDefinition.MotionProfileConfig motionProfile) {
    return new MechanismDefinition.ClosedLoopConfig(
        new MechanismDefinition.PIDConfig(gains.kP().get(), gains.kI().get(), gains.kD().get()),
        new MechanismDefinition.FeedforwardConfig(
            gains.kS().get(), gains.kG().get(), gains.kV().get(), gains.kA().get()),
        motionProfile,
        positionTolerance,
        velocityTolerance,
        maxVoltage);
  }

  private static MechanismDefinition.NeutralMode neutralMode(boolean brakeEnabled) {
    return brakeEnabled
        ? MechanismDefinition.NeutralMode.BRAKE
        : MechanismDefinition.NeutralMode.COAST;
  }

  private static MechanismDefinition.SoftLimitConfig softLimitsDisabled() {
    return softLimits(false, 0.0, 0.0);
  }

  private static MechanismDefinition.SoftLimitConfig softLimits(
      boolean enabled, double min, double max) {
    return new MechanismDefinition.SoftLimitConfig(enabled, min, max);
  }

  private static MechanismDefinition.SimulationConfig simulation(
      double reduction, double momentOfInertia, boolean gravityAware) {
    return new MechanismDefinition.SimulationConfig(
        true, reduction, momentOfInertia, gravityAware, true);
  }

  private static MechanismDefinition.KrakenFeatureConfig krakenFeaturesFor(
      MechanismDefinition.MotorControllerType controllerType) {
    if (!controllerType.supportsKrakenExtensions()) {
      return MechanismDefinition.KrakenFeatureConfig.disabled();
    }
    return new MechanismDefinition.KrakenFeatureConfig(true, true, false, 100, true);
  }

  private static MechanismDefinition.MotorControllerType motorController(
      IntakePivotConstants.MotorController controller) {
    return switch (controller) {
      case SPARK_MAX -> MechanismDefinition.MotorControllerType.SPARK_MAX;
      case SPARK_FLEX -> MechanismDefinition.MotorControllerType.SPARK_FLEX;
      case KRAKEN_X60, KRAKEN_X40 -> MechanismDefinition.MotorControllerType.TALON_FX;
    };
  }

  private static MechanismDefinition.MotorControllerType motorController(
      ShooterPivotConstants.MotorController controller) {
    return switch (controller) {
      case SPARK_MAX -> MechanismDefinition.MotorControllerType.SPARK_MAX;
      case SPARK_FLEX -> MechanismDefinition.MotorControllerType.SPARK_FLEX;
      case KRAKEN_X60, KRAKEN_X40 -> MechanismDefinition.MotorControllerType.TALON_FX;
    };
  }

  private static MechanismDefinition.MotorControllerType motorController(
      TurretConstants.MotorController controller) {
    return switch (controller) {
      case KRAKEN_X60, KRAKEN_X40 -> MechanismDefinition.MotorControllerType.TALON_FX;
    };
  }

  private static String canBusName(CANBus bus) {
    return bus.getName();
  }
}
