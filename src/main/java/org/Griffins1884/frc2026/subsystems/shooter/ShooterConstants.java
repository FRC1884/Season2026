package org.Griffins1884.frc2026.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class ShooterConstants {
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] SHOOTER_IDS = {20, 41};
  public static final boolean[] SHOOTER_INVERTED = {false, true};
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean BRAKE_MODE = false;
  public static final double REDUCTION = 1.0; // TODO: set gear ratio

  public static final double TARGET_RPM = 2000.0;

  public static final GlobalConstants.Gains gains =
      new GlobalConstants.Gains("Shooter/Gains", 0.001, 0.01, 1, 0.4621, 0.02, 0.010901);
  public static final double VELOCITY_TOLERANCE = 50; // TODO: tune for shooter
  public static final double MAX_VOLTAGE = 12.0; // TODO: tune for shooter

  public static final double FLYWHEEL_RADIUS_METERS = 0.05;
  public static final double FLYWHEEL_GEAR_RATIO = 1.0;
  public static final LoggedTunableNumber SLIP_FACTOR = new LoggedTunableNumber("Shooter/SlipFactor", 1.0); // TODO: tune for real exit speed
  public static final double EXIT_HEIGHT_METERS = 0.587;
  public static final double TARGET_HEIGHT_METERS = GlobalConstants.FieldConstants.Hub.innerHeight;
}
