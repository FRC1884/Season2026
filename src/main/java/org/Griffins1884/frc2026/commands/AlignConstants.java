package org.Griffins1884.frc2026.commands;

import org.Griffins1884.frc2026.GlobalConstants.Gains;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/** Shared constants and gain containers for drive alignment routines. */
public final class AlignConstants {
  private AlignConstants() {}

  public static final LoggedTunableNumber ALIGN_MAX_TRANSLATIONAL_SPEED =
      new LoggedTunableNumber("Align/MaxTranslationalSpeed", 1.0);
  public static final LoggedTunableNumber ALIGN_MAX_TRANSLATIONAL_ACCELERATION =
      new LoggedTunableNumber("Align/MaxTranslationalAcceleration", 2.0);
  public static final LoggedTunableNumber ALIGN_MAX_ANGULAR_SPEED =
      new LoggedTunableNumber("Align/MaxAngularSpeed", 0.3);
  public static final LoggedTunableNumber ALIGN_MAX_ANGULAR_ACCELERATION =
      new LoggedTunableNumber("Align/MaxAngularAcceleration", 0.3);
  public static final LoggedTunableNumber ALIGN_CONTROLLER_LOOP_PERIOD_SEC =
      new LoggedTunableNumber("Align/ControllerLoopPeriodSec", 0.02);
  public static final LoggedTunableNumber FF_START_DELAY =
      new LoggedTunableNumber("Align/FFStartDelaySec", 0.3);
  public static final LoggedTunableNumber FF_RAMP_RATE =
      new LoggedTunableNumber("Align/FFRampRateVoltsPerSec", 0.4);
  public static final LoggedTunableNumber ALIGN_MANUAL_DEADBAND =
      new LoggedTunableNumber("Align/ManualDeadband", 0.1);
  public static final LoggedTunableNumber ALIGN_TRANSLATION_TOLERANCE_METERS =
      new LoggedTunableNumber("Align/TranslationToleranceMeters", 0.2);
  public static final LoggedTunableNumber WHEEL_RADIUS_MAX_VELOCITY =
      new LoggedTunableNumber("Align/WheelRadiusMaxVelocity", 0.25);
  public static final LoggedTunableNumber WHEEL_RADIUS_RAMP_RATE =
      new LoggedTunableNumber("Align/WheelRadiusRampRate", 0.05);

  public static final LoggedTunableNumber ALIGN_TRANSLATION_KP =
      new LoggedTunableNumber("Align/Gains/Translation/kP", 0.4);
  public static final LoggedTunableNumber ALIGN_TRANSLATION_KI =
      new LoggedTunableNumber("Align/Gains/Translation/kI", 0.001);
  public static final LoggedTunableNumber ALIGN_TRANSLATION_KD =
      new LoggedTunableNumber("Align/Gains/Translation/kD", 0.4);
  public static final LoggedTunableNumber ALIGN_ROTATION_KP =
      new LoggedTunableNumber("Align/Gains/Rotation/kP", 2.0);
  public static final LoggedTunableNumber ALIGN_ROTATION_KI =
      new LoggedTunableNumber("Align/Gains/Rotation/kI", 0.0);
  public static final LoggedTunableNumber ALIGN_ROTATION_KD =
      new LoggedTunableNumber("Align/Gains/Rotation/kD", 0.0);
  public static final LoggedTunableNumber ALIGN_FEEDFORWARD_KV =
      new LoggedTunableNumber("Align/Gains/Feedforward/kV", 1.0);
  public static final LoggedTunableNumber ALIGN_FEEDFORWARD_DEADBAND =
      new LoggedTunableNumber("Align/Gains/Feedforward/DeadbandMeters", 0.02);

  public static AlignGains getAlignGains() {
    return new AlignGains(
        new Gains(
            ALIGN_TRANSLATION_KP.get(), ALIGN_TRANSLATION_KI.get(), ALIGN_TRANSLATION_KD.get()),
        new Gains(ALIGN_ROTATION_KP.get(), ALIGN_ROTATION_KI.get(), ALIGN_ROTATION_KD.get()),
        new FeedforwardGains(
            ALIGN_FEEDFORWARD_KV.get(),
            ALIGN_FEEDFORWARD_DEADBAND.get(),
            ALIGN_MAX_TRANSLATIONAL_SPEED.get()));
  }

  /** Feedforward definition for translation alignment. */
  public static record FeedforwardGains(
      double kV, double deadbandMeters, double maxSpeedMetersPerSecond) {
    public FeedforwardGains withKv(double value) {
      return new FeedforwardGains(value, deadbandMeters, maxSpeedMetersPerSecond);
    }
  }

  /**
   * Bundle of all alignment gains. X/Y are independent so we can tune strafe differently from
   * forward motion.
   */
  public static record AlignGains(
      Gains translationGains, Gains thetaGains, FeedforwardGains feedforwardGains) {
    public double translationKp() {
      return translationGains.kP();
    }

    public double translationKi() {
      return translationGains.kI();
    }

    public double translationKd() {
      return translationGains.kD();
    }

    public double rotationKp() {
      return thetaGains.kP();
    }

    public double rotationKi() {
      return thetaGains.kI();
    }

    public double rotationKd() {
      return thetaGains.kD();
    }

    public double feedforward() {
      return feedforwardGains.kV();
    }
  }
}
