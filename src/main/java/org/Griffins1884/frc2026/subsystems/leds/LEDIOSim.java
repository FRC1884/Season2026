package org.Griffins1884.frc2026.subsystems.leds;

import static edu.wpi.first.units.Units.Second;
import static org.Griffins1884.frc2026.subsystems.leds.LEDConstants.LED_LENGTH;
import static org.Griffins1884.frc2026.subsystems.leds.LEDConstants.LED_PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;

/**
 * Contains the methods that dictate simulated behavior for LEDs. <br>
 * <br>
 * Interestingly, you can read AddressableLED data directly from the sim GUI, without a roboRIO – no
 * need for AddressableLEDSim or the byte-conversion nonsense it warrants. Therefore, this is a
 * direct copy of LEDIOPWM.
 */
public class LEDIOSim implements LEDIO {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView[] views;

  private final LEDPattern[] patterns;
  private boolean enabled = true;

  public LEDIOSim() {
    AddressableLED ledLocal = null;
    AddressableLEDBuffer bufferLocal = null;
    AddressableLEDBufferView[] viewsLocal = null;
    LEDPattern[] patternsLocal = null;
    try {
      ledLocal = new AddressableLED(LED_PORT);
      bufferLocal = new AddressableLEDBuffer(LED_LENGTH);

      viewsLocal = new AddressableLEDBufferView[LEDConstants.SEGMENTS.length];
      patternsLocal = new LEDPattern[LEDConstants.SEGMENTS.length];

      for (int i = 0; i < LEDConstants.SEGMENTS.length; i++) {
        LEDConstants.Segment segment = LEDConstants.SEGMENTS[i];
        viewsLocal[i] =
            bufferLocal.createView(segment.start(), segment.start() + segment.length() - 1);

        if (segment.reversed()) {
          viewsLocal[i] = viewsLocal[i].reversed();
        }

        patternsLocal[i] = LEDPattern.solid(LEDConstants.IDLE_COLOR).breathe(Second.of(1));
        patternsLocal[i].applyTo(viewsLocal[i]);
      }

      ledLocal.setLength(bufferLocal.getLength());
      ledLocal.start();
    } catch (RuntimeException ex) {
      enabled = false;
      Logger.recordOutput("LED/DisabledReason", "InitFailure");
    }

    led = ledLocal;
    buffer = bufferLocal;
    views = viewsLocal;
    patterns = patternsLocal;
  }

  public void setPattern(int idx, LEDPattern pattern) {
    if (!enabled || pattern == null || views == null) {
      return;
    }
    try {
      pattern.applyTo(views[idx]);
    } catch (RuntimeException ex) {
      disableOutputs("SetPatternFailure");
    }
  }

  public void setPatterns(LEDPattern[] patterns) {
    if (!enabled || patterns == null || this.patterns == null) {
      return;
    }
    try {
      System.arraycopy(patterns, 0, this.patterns, 0, LEDConstants.SEGMENTS.length);
    } catch (RuntimeException ex) {
      disableOutputs("SetPatternsFailure");
    }
  }

  public void setAllPattern(LEDPattern pattern) {
    if (!enabled || this.patterns == null) {
      return;
    }
    try {
      for (int i = 0; i < LEDConstants.SEGMENTS.length; i++) {
        this.patterns[i] = pattern;
      }
    } catch (RuntimeException ex) {
      disableOutputs("SetAllPatternFailure");
    }
  }

  public void periodic() {
    if (!enabled || patterns == null || views == null || led == null) {
      return;
    }
    try {
      for (int i = 0; i < LEDConstants.SEGMENTS.length; i++) {
        patterns[i].applyTo(views[i]);
      }

      led.setData(buffer);
    } catch (RuntimeException ex) {
      disableOutputs("PeriodicFailure");
    }
  }

  @Override
  public void close() {
    if (led != null) {
      led.close();
    }
    enabled = false;
  }

  private void disableOutputs(String reason) {
    try {
      if (views != null && buffer != null && led != null) {
        LEDPattern off = LEDPattern.solid(Color.kBlack);
        for (AddressableLEDBufferView view : views) {
          if (view != null) {
            off.applyTo(view);
          }
        }
        led.setData(buffer);
      }
    } catch (RuntimeException ex) {
      // Ignore failures while attempting to clear LEDs.
    }
    enabled = false;
    Logger.recordOutput("LED/DisabledReason", reason);
  }
}
