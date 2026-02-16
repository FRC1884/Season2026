package org.Griffins1884.frc2026.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
  public static final int LED_PORT = 0;
  private static final Distance LED_SPACING = Meters.of(1 / 60.0); // 60 LEDs per meter
  public static int LED_LENGTH;

  public static Color IDLE_COLOR = Color.kAqua;
  public static Color INTAKE_SHOOT_COLOR = Color.kBlue;
  public static Color HAS_BALL_COLOR = Color.kGreen;
  public static Color FERRY_COLOR = Color.kYellow;
  public static Color CLIMB_DETACH_COLOR = Color.kRed;
  public static Color CLIMB_PULL_COLOR = Color.kBlue;
  public static Color CLIMB_DONE_COLOR = Color.kGreen;

  public static Color ALIGN_MORE_COLOR = Color.kGreen;
  public static Color ALIGN_LESS_COLOR = Color.kRed;
  public static Color ALIGN_OK_COLOR = Color.kBlue;

  public static Time BREATHE_SPEED = Second.of(2.5);
  public static Time CLIMB_BLINK_SPEED = Second.of(0.35);

  public static double TRANSLATION_TOLERANCE = 0.1;
  public static double ROTATION_TOLERANCE = 0.1;
  public static double FLASHING_MAX = 1;
  public static boolean NO_BALL_BREATHE = true;
  public static boolean HAS_BALL_SOLID = true;

  public static int LEFT = 0;
  public static int FRONT = 1;
  public static int RIGHT = 2;
  public static int BACK = 3;

  public static final Segment[] SEGMENTS =
      new Segment[] {
        new Segment(0, 50, false),
        new Segment(50, 50, false),
        new Segment(100, 50, false),
        new Segment(150, 50, false)
      };

  public record Segment(int start, int length, boolean reversed) {}

  static {
    for (Segment segment : SEGMENTS) LED_LENGTH += segment.length;
  }
}
