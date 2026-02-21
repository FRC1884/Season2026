package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.Griffins1884.frc2026.GlobalConstants.Gains;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/** Shared constants and gain containers for drive alignment routines. */
public final class AlignConstants {
  private AlignConstants() {}

  public static final LoggedTunableNumber ALIGN_MAX_TRANSLATIONAL_SPEED =
      new LoggedTunableNumber("Align/MaxTranslationalSpeed", 3.0);
  public static final LoggedTunableNumber ALIGN_MAX_TRANSLATIONAL_ACCELERATION =
      new LoggedTunableNumber("Align/MaxTranslationalAcceleration", 3.5);
  public static final LoggedTunableNumber ALIGN_MAX_ANGULAR_SPEED =
      new LoggedTunableNumber("Align/MaxAngularSpeed", 3.0);
  public static final LoggedTunableNumber ALIGN_MAX_ANGULAR_ACCELERATION =
      new LoggedTunableNumber("Align/MaxAngularAcceleration", 6.0);
  public static final LoggedTunableNumber ALIGN_CONTROLLER_LOOP_PERIOD_SEC =
      // The command scheduler runs at 20ms; treat this as the effective control loop period.
      new LoggedTunableNumber("Align/ControllerLoopPeriodSec", 0.02);
  public static final LoggedTunableNumber FF_START_DELAY =
      new LoggedTunableNumber("Align/FFStartDelaySec", 0.3);
  public static final LoggedTunableNumber FF_RAMP_RATE =
      new LoggedTunableNumber("Align/FFRampRateVoltsPerSec", 0.4);
  public static final LoggedTunableNumber ALIGN_MANUAL_DEADBAND =
      new LoggedTunableNumber("Align/ManualDeadband", 0.1);
  public static final LoggedTunableNumber ALIGN_TRANSLATION_TOLERANCE_METERS =
      new LoggedTunableNumber("Align/TranslationToleranceMeters", 0.03);
  public static final LoggedTunableNumber ALIGN_ROTATION_TOLERANCE_DEG =
      new LoggedTunableNumber("Align/RotationToleranceDeg", 2.0);
  public static final LoggedTunableNumber ALIGN_TOF_TOLERANCE_FRACTION =
      new LoggedTunableNumber("Align/TofToeranceFraction");
  public static final LoggedTunableNumber WHEEL_RADIUS_MAX_VELOCITY =
      new LoggedTunableNumber("Align/WheelRadiusMaxVelocity", 0.5);
  public static final LoggedTunableNumber WHEEL_RADIUS_RAMP_RATE =
      new LoggedTunableNumber("Align/WheelRadiusRampRate", 0.1);
  public static final Gains ALIGN_TRANSLATION_GAINS =
      new Gains("Align/Gains/Translation", 3, 0.001, 0.8);
  public static final Gains ALIGN_ROTATION_GAINS = new Gains("Align/Gains/Rotation", 6.0, 0.01, 2);
  public static final LoggedTunableNumber ALIGN_FEEDFORWARD_KV =
      new LoggedTunableNumber("Align/Gains/Feedforward/kV", 1.0);
  public static final LoggedTunableNumber ALIGN_FEEDFORWARD_DEADBAND =
      new LoggedTunableNumber("Align/Gains/Feedforward/DeadbandMeters", 0.02);

  public static final LoggedTunableNumber AFTER_COLLECT_START_X_METERS =
      new LoggedTunableNumber("Align/AfterCollectStart/XMeters", 7.7);
  public static final LoggedTunableNumber AFTER_COLLECT_START_Y_METERS =
      new LoggedTunableNumber("Align/AfterCollectStart/YMeters", 6.5);
  public static final LoggedTunableNumber AFTER_COLLECT_START_HEADING_DEG =
      new LoggedTunableNumber("Align/AfterCollectStart/HeadingDeg", 220.0);
  public static final LoggedTunableNumber TURRET_KV =
      new LoggedTunableNumber("Turret/AutoAim/kV", 1);
  public static final LoggedTunableNumber TURRET_KS =
      new LoggedTunableNumber("Turret/AutoAim/kS", 0);

  public static final LoggedTunableNumber AFTER_BUMP_START_X_METERS =
      new LoggedTunableNumber("Align/AfterBumpStart/XMeters", 7.7);
  public static final LoggedTunableNumber AFTER_BUMP_START_Y_METERS =
      new LoggedTunableNumber("Align/AfterBumpStart/YMeters", 0.9);
  public static final LoggedTunableNumber AFTER_BUMP_START_HEADING_DEG =
      new LoggedTunableNumber("Align/AfterBumpStart/HeadingDeg", -90);

  public static final LoggedTunableNumber AFTER_SECOND_BUMP_START_X_METERS =
      new LoggedTunableNumber("Align/AfterSecondBump/XMeters", 2);
  public static final LoggedTunableNumber AFTER_SECOND_BUMP_START_Y_METERS =
      new LoggedTunableNumber("Align/AfterSecondBump/YMeters", 4);
  public static final LoggedTunableNumber AFTER_SECOND_BUMP_START_HEADING_DEG =
      new LoggedTunableNumber("Align/AfterSecondBump/HeadingDeg", 0);

  public static final LoggedTunableNumber AFTER_BUMP_TO_NEUTRAL_START_X_METERS =
      new LoggedTunableNumber("Align/AfterBumpToNeutral/XMeters", 6.5);
  public static final LoggedTunableNumber AFTER_BUMP_TO_NEUTRAL_START_Y_METERS =
      new LoggedTunableNumber("Align/AfterBumpToNeutral/YMeters", 5.6);
  public static final LoggedTunableNumber AFTER_BUMP_TO_NEUTRAL_START_HEADING_DEG =
      new LoggedTunableNumber("Align/AfterBumpToNeutral/HeadingDeg", 180);

  public static AlignGains getAlignGains() {
    return new AlignGains(
        ALIGN_TRANSLATION_GAINS,
        ALIGN_ROTATION_GAINS,
        new FeedforwardGains(
            ALIGN_FEEDFORWARD_KV.get(),
            ALIGN_FEEDFORWARD_DEADBAND.get(),
            ALIGN_MAX_TRANSLATIONAL_SPEED.get()));
  }

  public static Pose2d getAfterCollectStartPose() {
    return new Pose2d(
        new Translation2d(AFTER_COLLECT_START_X_METERS.get(), AFTER_COLLECT_START_Y_METERS.get()),
        Rotation2d.fromDegrees(AFTER_COLLECT_START_HEADING_DEG.get()));
  }

  public static Pose2d getAfterOverBumpStartPose() {
    return new Pose2d(
        new Translation2d(AFTER_BUMP_START_X_METERS.get(), AFTER_BUMP_START_Y_METERS.get()),
        Rotation2d.fromDegrees(AFTER_BUMP_START_HEADING_DEG.get()));
  }

  public static Pose2d getAfterSecondBumpStartPose() {
    return new Pose2d(
        new Translation2d(
            AFTER_SECOND_BUMP_START_X_METERS.get(), AFTER_SECOND_BUMP_START_Y_METERS.get()),
        Rotation2d.fromDegrees(AFTER_SECOND_BUMP_START_HEADING_DEG.get()));
  }

  public static Pose2d getAfterBumpToNeutralStartPose() {
    return new Pose2d(
        new Translation2d(
            AFTER_BUMP_TO_NEUTRAL_START_X_METERS.get(), AFTER_BUMP_TO_NEUTRAL_START_Y_METERS.get()),
        Rotation2d.fromDegrees(AFTER_BUMP_TO_NEUTRAL_START_HEADING_DEG.get()));
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
      return translationGains.kP().get();
    }

    public double translationKi() {
      return translationGains.kI().get();
    }

    public double translationKd() {
      return translationGains.kD().get();
    }

    public double rotationKp() {
      return thetaGains.kP().get();
    }

    public double rotationKi() {
      return thetaGains.kI().get();
    }

    public double rotationKd() {
      return thetaGains.kD().get();
    }

    public double feedforward() {
      return feedforwardGains.kV();
    }
  }
}
