package frc.robot.subsystems.shooter;

import frc.robot.GlobalConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class ShooterConstants {
  public static final int SHOOTER_ID = 62; // TODO: Find ID for actual intake
  public static final boolean isFlex = true;
  public static final DoubleSupplier currentLimit =
      new LoggedTunableNumber("CoralIntake/CurrentLimit", 15);
  public static final GlobalConstants.Gains gains = new GlobalConstants.Gains(1, 0, 0);
  public static final double VELOCITY_TOLERANCE = 0.0; // TODO: tune for shooter
  public static final double MAX_VOLTAGE = 12.0; // TODO: tune for shooter
}
