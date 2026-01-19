package org.Griffins1884.frc2026.subsystems.exampleClasses.shooter;

import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class ShooterConstants {
  public static final int INTAKE_ALGAE_ID = 62; // TODO: Find ID for actual intake
  public static final boolean isFlex = true;
  public static final DoubleSupplier currentLimit =
      new LoggedTunableNumber("CoralIntake/CurrentLimit", 15);
  public static final GlobalConstants.Gains gains = new GlobalConstants.Gains(1, 0, 0);
  public static final double VELOCITY_TOLERANCE = 0.0; // TODO: tune for shooter
  public static final double MAX_VOLTAGE = 12.0; // TODO: tune for shooter
}
