package org.Griffins1884.frc2026.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

/**
 * This class contains "common" behavior between real and simulated LEDs. Stuff we want to see in
 * both modes, like setting the whole strip to one color or setting individual LEDs, go in here.
 */
public interface LEDIO {
  /** LEDs don't read any sensors, so this inner class should be left empty. */
  @AutoLog
  class LEDIOInputs {
    // No sensors so no inputs
  }

  default void setPattern(int idx, LEDPattern pattern) {}

  default void setPatterns(LEDPattern[] patterns) {}

  default void setAllPattern(LEDPattern pattern) {}

  public default void periodic() {}
}
