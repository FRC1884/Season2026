package org.Griffins1884.frc2026;

import static org.Griffins1884.frc2026.GlobalConstants.ROBOT;

import org.Griffins1884.frc2026.OI.DriverMap;
import org.Griffins1884.frc2026.OI.SimXboxUniversalMap;
import org.Griffins1884.frc2026.OI.XboxDriverMap;

public final class Config {

  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean AUTONOMOUS_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean LEDS_ENABLED = false;
    public static final boolean WEBUI_ENABLED = true;
    public static final boolean TURRET_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean SHOOTER_PIVOT_ENABLED = true;
    public static final boolean INTAKE_PIVOT_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean INDEXER_ENABLED = true;
  }

  public static final class WebUIConfig {
    private static final String DEFAULT_BIND_ADDRESS = "0.0.0.0";
    private static final int DEFAULT_PORT = 5805;

    public static final boolean ENABLED = Subsystems.WEBUI_ENABLED;
    public static final String BIND_ADDRESS = DEFAULT_BIND_ADDRESS;
    public static final int PORT = DEFAULT_PORT;
  }

  public static final class Controllers {
    public static final int DRIVER_PORT = 0;

    public static DriverMap getDriverController() {
      return switch (ROBOT) {
        case COMPBOT -> new XboxDriverMap(DRIVER_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }
  }
}
