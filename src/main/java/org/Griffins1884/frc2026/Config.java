package org.Griffins1884.frc2026;

import static org.Griffins1884.frc2026.GlobalConstants.ROBOT;

import org.Griffins1884.frc2026.OI.BoardOperatorMap;
import org.Griffins1884.frc2026.OI.DriverMap;
import org.Griffins1884.frc2026.OI.OperatorMap;
import org.Griffins1884.frc2026.OI.PS5DriverMap;
import org.Griffins1884.frc2026.OI.SimXboxUniversalMap;
import org.Griffins1884.frc2026.OI.XboxDriverMap;
import org.Griffins1884.frc2026.OI.XboxOperatorMap;

public final class Config {

  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean AUTONOMOUS_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean LEDS_ENABLED = true;
    public static final boolean WEBUI_ENABLED = true;
    public static final boolean TURRET_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean SHOOTER_PIVOT_ENABLED = true;
    public static final boolean INTAKE_PIVOT_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean INDEXER_ENABLED = true;
    public static final boolean CLIMBER_ENABLED = true;
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

    public static final int OPERATOR_PORT = 1;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false;

    public static DriverMap getDriverController() {
      return switch (ROBOT) {
        case COMPBOT -> new XboxDriverMap(DRIVER_PORT);
        case CRESCENDO -> new PS5DriverMap(DRIVER_PORT);
        case DEVBOT -> new XboxDriverMap(DRIVER_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }

    public static OperatorMap getOperatorController() {
      return switch (ROBOT) {
        case COMPBOT, CRESCENDO, DEVBOT ->
            JOYSTICK_OPERATOR_ENABLED
                ? new XboxOperatorMap(OPERATOR_PORT)
                : new BoardOperatorMap(OPERATOR_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }
  }
}
