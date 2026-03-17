package org.Griffins1884.frc2026.commands;

import org.Griffins1884.frc2026.GlobalConstants.Gains;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/** Shared tuning values for the remaining drivetrain and turret alignment flows. */
public final class AlignConstants {
  private AlignConstants() {}

  // The command scheduler runs at 20 ms; treat this as the effective control-loop period.
  public static final double LOOP_PERIOD_SEC = 0.02;

  public static final class Manual {
    public static final LoggedTunableNumber DEADBAND =
        new LoggedTunableNumber("Align/ManualDeadband", 0.1);

    private Manual() {}
  }

  public static final class Auto {
    public static final LoggedTunableNumber MAX_LINEAR_SPEED_MPS =
        new LoggedTunableNumber("Align/MaxTranslationalSpeed", 3.0);
    public static final LoggedTunableNumber MAX_LINEAR_ACCEL_MPS2 =
        new LoggedTunableNumber("Align/MaxTranslationalAcceleration", 3.5);
    public static final LoggedTunableNumber MAX_ANGULAR_SPEED_RAD_PER_SEC =
        new LoggedTunableNumber("Align/MaxAngularSpeed", 3.0);
    public static final LoggedTunableNumber MAX_ANGULAR_ACCEL_RAD_PER_SEC2 =
        new LoggedTunableNumber("Align/MaxAngularAcceleration", 6.0);
    public static final LoggedTunableNumber TRANSLATION_TOLERANCE_METERS =
        new LoggedTunableNumber("Align/TranslationToleranceMeters", 0.03);
    public static final LoggedTunableNumber ROTATION_TOLERANCE_DEG =
        new LoggedTunableNumber("Align/RotationToleranceDeg", 2.0);
    public static final Gains TRANSLATION_GAINS =
        new Gains("Align/Gains/Translation", 3.0, 0.001, 0.8);
    public static final Gains ROTATION_GAINS = new Gains("Align/Gains/Rotation", 6.0, 0.01, 2.0);
    public static final LoggedTunableNumber FEEDFORWARD_KV =
        new LoggedTunableNumber("Align/Gains/Feedforward/kV", 1.0);
    public static final LoggedTunableNumber FEEDFORWARD_DEADBAND_METERS =
        new LoggedTunableNumber("Align/Gains/Feedforward/DeadbandMeters", 0.02);

    private Auto() {}
  }

  public static final class Characterization {
    public static final LoggedTunableNumber FF_START_DELAY_SEC =
        new LoggedTunableNumber("Align/FFStartDelaySec", 0.3);
    public static final LoggedTunableNumber FF_RAMP_RATE_VOLTS_PER_SEC =
        new LoggedTunableNumber("Align/FFRampRateVoltsPerSec", 0.4);
    public static final LoggedTunableNumber WHEEL_RADIUS_MAX_VELOCITY_RAD_PER_SEC =
        new LoggedTunableNumber("Align/WheelRadiusMaxVelocity", 0.5);
    public static final LoggedTunableNumber WHEEL_RADIUS_RAMP_RATE_RAD_PER_SEC2 =
        new LoggedTunableNumber("Align/WheelRadiusRampRate", 0.1);

    private Characterization() {}
  }

  public static final class TurretAutoAim {
    public static final LoggedTunableNumber TOF_TOLERANCE_FRACTION =
        // Keep legacy key spelling ("Toerance") for dashboard compatibility.
        new LoggedTunableNumber("Align/TofToeranceFraction", 0.01);
    public static final LoggedTunableNumber KV =
        new LoggedTunableNumber("Turret/AutoAim/kV", 1.22, true);
    public static final LoggedTunableNumber KS =
        new LoggedTunableNumber("Turret/AutoAim/kS", -0.005, true);
    public static final LoggedTunableNumber BASE_LATENCY_SECONDS =
        new LoggedTunableNumber("Turret/AutoAim/BaseLatencySeconds", 0.2, true);
    public static final LoggedTunableNumber MAX_MOTION_SAMPLE_AGE_SECONDS =
        new LoggedTunableNumber("Turret/AutoAim/MaxMotionSampleAgeSeconds", 0.15, true);
    public static final LoggedTunableNumber MAX_MOTION_SPEED_MPS =
        new LoggedTunableNumber("Turret/AutoAim/MaxMotionSpeedMps", 6.0, true);
    public static final LoggedTunableNumber MAX_MOTION_ACCEL_MPS2 =
        new LoggedTunableNumber("Turret/AutoAim/MaxMotionAccelMps2", 18.0, true);

    private TurretAutoAim() {}
  }
}
