package org.Griffins1884.frc2026.simulation.maple;

import edu.wpi.first.math.util.Units;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;

/** Season2026-owned chassis envelope and mass properties for the rebuilt field simulation. */
public final class Season2026RobotProfile {
  public static final ChassisFootprint DEFAULT_FOOTPRINT =
      new ChassisFootprint(
          Units.inchesToMeters(34.0),
          Units.inchesToMeters(34.0),
          Units.inchesToMeters(21.75),
          Units.inchesToMeters(1.5));

  public static final ChassisMassProperties DEFAULT_CHASSIS_MASS_PROPERTIES =
      new ChassisMassProperties(
          61.235,
          Units.inchesToMeters(11.5),
          Units.inchesToMeters(27.5),
          Units.inchesToMeters(27.5),
          1.1);

  private Season2026RobotProfile() {}
}
