package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.AllianceFlipUtil;
import org.Griffins1884.frc2026.util.RobotLogging;

public class DriveCommands {
  private DriveCommands() {}

  private static boolean test = false;

  public static Supplier<Boolean> getTest() {
    return () -> test;
  }

  public static void setTest(boolean newTest) {
    test = newTest;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), AlignConstants.Manual.DEADBAND.get());
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static void joystickDrive(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), AlignConstants.Manual.DEADBAND.get());

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());
    boolean isFlipped = AllianceFlipUtil.shouldFlip(drive.getPose());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  public static Command joystickDriveCommand(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(() -> joystickDrive(drive, xSupplier, ySupplier, omegaSupplier), drive);
  }

  /**
   * Robot-relative drive command with driver controls flipped so the back of the robot behaves as
   * the front while the override is held.
   */
  public static Command joystickDriveRobotRelativeFlippedCommand(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              MathUtil.applyDeadband(
                  omegaSupplier.getAsDouble(), AlignConstants.Manual.DEADBAND.get());
          omega = Math.copySign(omega * omega, omega);

          drive.runVelocity(
              new ChassisSpeeds(
                  -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  -omega * drive.getMaxAngularSpeedRadPerSec()));
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(SwerveSubsystem drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(AlignConstants.Characterization.FF_START_DELAY_SEC.get()),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage =
                      timer.get()
                          * AlignConstants.Characterization.FF_RAMP_RATE_VOLTS_PER_SEC.get();
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  RobotLogging.debug("********** Drive FF Characterization Results **********");
                  RobotLogging.debug("\tkS: " + formatter.format(kS));
                  RobotLogging.debug("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter =
        new SlewRateLimiter(
            AlignConstants.Characterization.WHEEL_RADIUS_RAMP_RATE_RAD_PER_SEC2.get());
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed =
                      limiter.calculate(
                          AlignConstants.Characterization.WHEEL_RADIUS_MAX_VELOCITY_RAD_PER_SEC
                              .get());
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      RobotLogging.debug(
                          "********** Wheel Radius Characterization Results **********");
                      RobotLogging.debug(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      RobotLogging.debug(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      RobotLogging.debug(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
