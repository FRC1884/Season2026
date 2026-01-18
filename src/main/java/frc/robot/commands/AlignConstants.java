package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Shared constants and gain containers for drive alignment routines. */
public final class AlignConstants {
  private AlignConstants() {}

  public static final double ALIGN_MAX_TRANSLATIONAL_SPEED = 5.0; // m/s
  public static final double ALIGN_MAX_TRANSLATIONAL_ACCELERATION = 8.0; // m/s
  public static final double ALIGN_MAX_ANGULAR_SPEED = 6.0; // m/s
  public static final double ALIGN_MAX_ANGULAR_ACCELERATION = 20.0; // m/s^2
  public static final double ALIGN_CONTROLLER_LOOP_PERIOD_SEC = 0.02; // PathPlanner controller dt
  public static final double FF_START_DELAY = 0.3; // Secs
  public static final double FF_RAMP_RATE = 0.4; // Volts/Sec
  public static final double ALIGN_TRANSLATION_SLEW_RATE = 6.0; // m/s^2
  public static final double ALIGN_ROTATION_SLEW_RATE = Math.toRadians(720.0); // rad/s^2
  public static final double ALIGN_MANUAL_MAX_SPEED = 5.0; // m/s
  public static final double ALIGN_MANUAL_DEADBAND = 0.1;
  public static final double ALIGN_PID_BLEND_RANGE_METERS = 1.0; // distance where PID fully enabled
  public static final double ALIGN_PID_BLEND_MIN = 0.15; // keep some PID authority near target
  public static final double ALIGN_TRANSLATION_TOLERANCE_METERS = 0.03; // m
  public static final double ALIGN_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(1.5); // rad
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static final TrapezoidProfile.Constraints ALIGN_ROTATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Math.toRadians(540.0), Math.toRadians(900.0));

  public static final AlignGains DEFAULT_ALIGN_GAINS =
      new AlignGains(
          new PIDGains(5.5, 0.1, 1.0),
          new PIDGains(5.5, 0.1, 1.0),
          new PIDGains(10.0, 0.0, 3.0),
          new FeedforwardGains(6.0, 0.02, ALIGN_MAX_TRANSLATIONAL_SPEED));

  public static final double ALIGN_TUNER_TIMEOUT_SECS = 3.5;
  public static final double ALIGN_TUNER_KICKBACK_TIME = 0.6;
  public static final double ALIGN_TUNER_KICKBACK_SPEED = 1.5;
  public static final int ALIGN_TUNER_TRIAL_COUNT = 3;

  /** Simple PID container that mirrors the WPILib controller parameters. */
  public static record PIDGains(double kP, double kI, double kD) {
    public PIDGains withKp(double value) {
      return new PIDGains(value, kI, kD);
    }

    public PIDGains withKd(double value) {
      return new PIDGains(kP, kI, value);
    }
  }

  /** Feedforward definition for translation alignment. */
  public static record FeedforwardGains(
      double kV, double deadbandMeters, double maxSpeedMetersPerSecond) {
    public FeedforwardGains withKv(double value) {
      return new FeedforwardGains(value, deadbandMeters, maxSpeedMetersPerSecond);
    }

    public double calculateSpeed(double distanceMeters) {
      if (distanceMeters < deadbandMeters) {
        return 0.0;
      }
      return MathUtil.clamp(distanceMeters * kV, 0.0, maxSpeedMetersPerSecond);
    }
  }

  /**
   * Bundle of all alignment gains. X/Y are independent so we can tune strafe differently from
   * forward motion.
   */
  public static record AlignGains(
      PIDGains xGains, PIDGains yGains, PIDGains thetaGains, FeedforwardGains feedforwardGains) {
    public double translationKp() {
      return xGains.kP();
    }

    public double translationKi() {
      return xGains.kI();
    }

    public double translationKd() {
      return xGains.kD();
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

    public PIDConstants translationPidConstants() {
      return new PIDConstants(xGains.kP(), xGains.kI(), xGains.kD());
    }

    public PIDConstants rotationPidConstants() {
      return new PIDConstants(thetaGains.kP(), thetaGains.kI(), thetaGains.kD());
    }

    public AlignGains withTranslationKp(double kp) {
      return new AlignGains(xGains.withKp(kp), yGains.withKp(kp), thetaGains, feedforwardGains);
    }

    public AlignGains withRotationKp(double kp) {
      return new AlignGains(xGains, yGains, thetaGains.withKp(kp), feedforwardGains);
    }

    public AlignGains withFeedforward(double kv) {
      return new AlignGains(xGains, yGains, thetaGains, feedforwardGains.withKv(Math.max(0.0, kv)));
    }
  }
}
