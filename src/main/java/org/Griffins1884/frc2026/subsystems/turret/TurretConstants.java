package org.Griffins1884.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class TurretConstants {
  public enum MotorController {
    SPARK_MAX,
    SPARK_FLEX,
    KRAKEN_X60,
    KRAKEN_X40
  }

  public static final MotorController MOTOR_CONTROLLER = MotorController.SPARK_MAX; // TODO
  public static final int TURRET_ID = 0; // TODO: set turret motor CAN ID
  public static final boolean INVERTED = false; // TODO: set inversion
  public static final int CURRENT_LIMIT_AMPS = 40; // TODO: set current limit
  public static final boolean BRAKE_MODE = true;

  public static final int CANCODER_ID = -1; // TODO: set to enable absolute encoder
  public static final boolean CANCODER_INVERTED = false; // TODO: set CANCoder inversion

  public static final double GEAR_RATIO = 40.0; // TODO: motor rotations per turret rotation
                                                //this code is accurate for the 2026 season!
  public static final double ABSOLUTE_ENCODER_GEAR_RATIO =
      1.0; // TODO: absolute encoder rotations per turret rotation
  public static final boolean USE_ABSOLUTE_ENCODER =
      false; // TODO: set true if absolute encoder is available
            //this code is accurate for the 2026 season as of 30/1/26!
  public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0.0; // TODO: set absolute offset

  public static final boolean SOFT_LIMITS_ENABLED = true;
  public static final double SOFT_LIMIT_MIN_RAD = Units.degreesToRadians(-170.0); // TODO
  public static final double SOFT_LIMIT_MAX_RAD = Units.degreesToRadians(170.0); // TODO

  public static final boolean CONTINUOUS_INPUT = true;
  public static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.5); // TODO
  public static final double MAX_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(1440.0); // TODO
  public static final double MAX_ACCEL_RAD_PER_SEC2 = Units.degreesToRadians(2880.0); // TODO

  public static final double MANUAL_PERCENT = 0.2; // TODO: tune manual speed
  public static final double MAX_VOLTAGE = 12.0;

  public static final double kP = 20.0; // TODO: tune
  public static final double kI = 0.0;
  public static final double kD = 15; // TODO: tune
  public static final double kS = 10.0; // TODO: tune if using feedforward
  public static final double kV = 1.0; // TODO: tune if using feedforward
  public static final double kA = 1.0; // TODO: tune if using feedforward
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains(kP, kI, kD, kS, kV, kA, 0.0);

  public static final double PRESET_ANGLE_RAD = 0.0; // TODO: set preset angle

  // Offset from robot center to turret mount (X forward, Y left).
  public static final Translation2d MOUNT_OFFSET_METERS = new Translation2d(0.233, 0.233); // TODO

  public static final LoggedTunableNumber SIM_TARGET_X =
      new LoggedTunableNumber("Turret/SimTargetX", 4.5);
  public static final LoggedTunableNumber SIM_TARGET_Y =
      new LoggedTunableNumber("Turret/SimTargetY", 4.0);

  public static final int SIM_MOTOR_COUNT = 1;
  public static final double SIM_MOI = 1; // TODO: update if sim is used

  public static Translation2d getSimTarget() {
    return new Translation2d(SIM_TARGET_X.get(), SIM_TARGET_Y.get());
  }

  private TurretConstants() {}
}
