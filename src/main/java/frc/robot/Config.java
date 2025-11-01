package frc.robot;

import static frc.robot.GlobalConstants.ROBOT;

import frc.robot.OI.BoardOperatorMap;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.OI.SimXboxUniversalMap;
import frc.robot.OI.XboxDriverMap;
import frc.robot.OI.XboxOperatorMap;

public final class Config {

  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean AUTONOMOUS_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean LEDS_ENABLED = false;
    public static final boolean WEBUI_ENABLED = false;
    public static final boolean IsSwerveSpark = true;
  }

  public static final class WebUIConfig {
    private static final String DEFAULT_BIND_ADDRESS = "0.0.0.0";
    private static final int DEFAULT_PORT = 5805;

    public static final boolean ENABLED = Subsystems.WEBUI_ENABLED;
    public static final String BIND_ADDRESS =
        getStringConfig("webui.bindAddress", "WEBUI_BIND_ADDRESS", DEFAULT_BIND_ADDRESS);
    public static final int PORT = getIntConfig("webui.port", "WEBUI_PORT", DEFAULT_PORT);

    private static String getStringConfig(String propertyKey, String envKey, String fallback) {
      String property = System.getProperty(propertyKey);
      if (property != null && !property.isBlank()) {
        return property;
      }
      String env = System.getenv(envKey);
      if (env != null && !env.isBlank()) {
        return env;
      }
      return fallback;
    }

    private static int getIntConfig(String propertyKey, String envKey, int fallback) {
      String raw = System.getProperty(propertyKey);
      if (raw == null || raw.isBlank()) {
        raw = System.getenv(envKey);
      }
      if (raw == null || raw.isBlank()) {
        return fallback;
      }
      try {
        return Integer.parseInt(raw.trim());
      } catch (NumberFormatException ex) {
        System.err.println(
            "Invalid WebUI port '" + raw + "', falling back to default " + fallback + ".");
        return fallback;
      }
    }
  }

  public static final class Controllers {
    public static final int DRIVER_PORT = 0;

    public static final int OPERATOR_PORT = 1;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false;

    public static DriverMap getDriverController() {
      return switch (ROBOT) {
        case COMPBOT -> new XboxDriverMap(DRIVER_PORT);
        case DEVBOT -> new XboxDriverMap(DRIVER_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }

    public static OperatorMap getOperatorController() {
      return switch (ROBOT) {
        case COMPBOT -> JOYSTICK_OPERATOR_ENABLED
            ? new XboxOperatorMap(OPERATOR_PORT)
            : new BoardOperatorMap(OPERATOR_PORT);
        case DEVBOT -> JOYSTICK_OPERATOR_ENABLED
            ? new XboxOperatorMap(OPERATOR_PORT)
            : new BoardOperatorMap(OPERATOR_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }
  }
}
