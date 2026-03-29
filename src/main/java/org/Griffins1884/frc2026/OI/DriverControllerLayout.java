package org.Griffins1884.frc2026.OI;

import edu.wpi.first.wpilibj.GenericHID;
import java.util.Locale;
import java.util.Map;

enum DriverControllerLayout {
  XBOX(
      "xbox",
      "Xbox",
      Map.of(
          "left-x", new AxisSpec(0, -1.0, 0.08),
          "left-y", new AxisSpec(1, -1.0, 0.08),
          "right-x", new AxisSpec(4, -1.0, 0.08),
          "right-y", new AxisSpec(5, -1.0, 0.08),
          "l2", new AxisSpec(2, 1.0, 0.5),
          "r2", new AxisSpec(3, 1.0, 0.5)),
      Map.ofEntries(
          Map.entry("south", new ButtonSpec(1)),
          Map.entry("east", new ButtonSpec(2)),
          Map.entry("west", new ButtonSpec(3)),
          Map.entry("north", new ButtonSpec(4)),
          Map.entry("l1", new ButtonSpec(5)),
          Map.entry("r1", new ButtonSpec(6)),
          Map.entry("back", new ButtonSpec(7)),
          Map.entry("menu", new ButtonSpec(8)),
          Map.entry("left-stick-press", new ButtonSpec(9)),
          Map.entry("right-stick-press", new ButtonSpec(10)),
          Map.entry("home", new ButtonSpec(11))),
      Map.of(
          "dpad-up", new PovSpec(0),
          "dpad-right", new PovSpec(90),
          "dpad-down", new PovSpec(180),
          "dpad-left", new PovSpec(270))),
  PS4(
      "ps4",
      "PS4",
      Map.of(
          "left-x", new AxisSpec(0, -1.0, 0.08),
          "left-y", new AxisSpec(1, -1.0, 0.08),
          "right-x", new AxisSpec(2, -1.0, 0.08),
          "right-y", new AxisSpec(5, -1.0, 0.08),
          "l2", new AxisSpec(3, 1.0, 0.5),
          "r2", new AxisSpec(4, 1.0, 0.5)),
      Map.ofEntries(
          Map.entry("west", new ButtonSpec(1)),
          Map.entry("south", new ButtonSpec(2)),
          Map.entry("east", new ButtonSpec(3)),
          Map.entry("north", new ButtonSpec(4)),
          Map.entry("l1", new ButtonSpec(5)),
          Map.entry("r1", new ButtonSpec(6)),
          Map.entry("back", new ButtonSpec(9)),
          Map.entry("menu", new ButtonSpec(10)),
          Map.entry("left-stick-press", new ButtonSpec(11)),
          Map.entry("right-stick-press", new ButtonSpec(12)),
          Map.entry("home", new ButtonSpec(13)),
          Map.entry("touchpad", new ButtonSpec(14))),
      Map.of(
          "dpad-up", new PovSpec(0),
          "dpad-right", new PovSpec(90),
          "dpad-down", new PovSpec(180),
          "dpad-left", new PovSpec(270))),
  PS5_PRO(
      "ps5-pro",
      "PS5 Pro",
      Map.of(
          "left-x", new AxisSpec(0, -1.0, 0.08),
          "left-y", new AxisSpec(1, -1.0, 0.08),
          "right-x", new AxisSpec(2, -1.0, 0.08),
          "right-y", new AxisSpec(5, -1.0, 0.08),
          "l2", new AxisSpec(3, 1.0, 0.5),
          "r2", new AxisSpec(4, 1.0, 0.5)),
      Map.ofEntries(
          Map.entry("west", new ButtonSpec(1)),
          Map.entry("south", new ButtonSpec(2)),
          Map.entry("east", new ButtonSpec(3)),
          Map.entry("north", new ButtonSpec(4)),
          Map.entry("l1", new ButtonSpec(5)),
          Map.entry("r1", new ButtonSpec(6)),
          Map.entry("back", new ButtonSpec(9)),
          Map.entry("menu", new ButtonSpec(10)),
          Map.entry("left-stick-press", new ButtonSpec(11)),
          Map.entry("right-stick-press", new ButtonSpec(12)),
          Map.entry("home", new ButtonSpec(13)),
          Map.entry("touchpad", new ButtonSpec(14)),
          Map.entry("left-paddle", new ButtonSpec(15)),
          Map.entry("right-paddle", new ButtonSpec(16))),
      Map.of(
          "dpad-up", new PovSpec(0),
          "dpad-right", new PovSpec(90),
          "dpad-down", new PovSpec(180),
          "dpad-left", new PovSpec(270)));

  private final String profileType;
  private final String displayName;
  private final Map<String, AxisSpec> axes;
  private final Map<String, ButtonSpec> buttons;
  private final Map<String, PovSpec> povButtons;

  DriverControllerLayout(
      String profileType,
      String displayName,
      Map<String, AxisSpec> axes,
      Map<String, ButtonSpec> buttons,
      Map<String, PovSpec> povButtons) {
    this.profileType = profileType;
    this.displayName = displayName;
    this.axes = axes;
    this.buttons = buttons;
    this.povButtons = povButtons;
  }

  String profileType() {
    return profileType;
  }

  String displayName() {
    return displayName;
  }

  double readAxis(GenericHID hid, String inputId) {
    AxisSpec axis = axes.get(canonicalInputId("axis", inputId));
    return axis == null ? 0.0 : axis.read(hid);
  }

  boolean readInputAsBoolean(GenericHID hid, String inputKind, String inputId) {
    String canonicalKind = normalizeInputKind(inputKind);
    String canonicalInputId = canonicalInputId(canonicalKind, inputId);
    return switch (canonicalKind) {
      case "axis" -> {
        AxisSpec axis = axes.get(canonicalInputId);
        yield axis != null && axis.isPressed(hid);
      }
      case "button" -> {
        ButtonSpec button = buttons.get(canonicalInputId);
        if (button != null) {
          yield button.isPressed(hid);
        }
        PovSpec pov = povButtons.get(canonicalInputId);
        yield pov != null && pov.isPressed(hid);
      }
      case "pov" -> {
        PovSpec pov = povButtons.get(canonicalInputId);
        yield pov != null && pov.isPressed(hid);
      }
      default -> false;
    };
  }

  String canonicalInputId(String inputKind, String inputId) {
    String normalized = normalizeToken(inputId);
    if (normalized.isEmpty()) {
      return normalized;
    }
    return switch (normalized) {
      case "a", "cross" -> "south";
      case "b", "circle" -> "east";
      case "x", "square" -> "west";
      case "y", "triangle" -> "north";
      case "lb", "leftbumper", "left-bumper" -> "l1";
      case "rb", "rightbumper", "right-bumper" -> "r1";
      case "lt", "lefttrigger", "left-trigger" -> "l2";
      case "rt", "righttrigger", "right-trigger" -> "r2";
      case "share", "view", "create" -> "back";
      case "start", "options" -> "menu";
      case "l3", "leftstickpress", "left-stick-button" -> "left-stick-press";
      case "r3", "rightstickpress", "right-stick-button" -> "right-stick-press";
      case "ps" -> "home";
      case "touch-pad" -> "touchpad";
      case "paddleleft", "leftbackbutton", "left-back-button" -> "left-paddle";
      case "paddleright", "rightbackbutton", "right-back-button" -> "right-paddle";
      default -> normalized;
    };
  }

  static String normalizeInputKind(String inputKind) {
    String normalized = normalizeToken(inputKind);
    if (normalized.equals("trigger")) {
      return "axis";
    }
    return normalized;
  }

  static DriverControllerLayout fromProfileType(String profileType) {
    String normalized = normalizeToken(profileType);
    return switch (normalized) {
      case "xbox", "simxboxuniversal", "sim-xbox-universal" -> XBOX;
      case "ps4", "dualshock4", "dualshock-4" -> PS4;
      case "ps5",
              "ps5pro",
              "ps5-pro",
              "dualsense",
              "dualsenseedge",
              "dualsense-edge",
              "genericgamepad" ->
          PS5_PRO;
      default -> PS5_PRO;
    };
  }

  private static String normalizeToken(String value) {
    return value == null
        ? ""
        : value.trim().toLowerCase(Locale.ROOT).replace('_', '-').replace(' ', '-');
  }

  private record AxisSpec(int axisIndex, double scale, double pressThreshold) {
    private double read(GenericHID hid) {
      double value = hid.getRawAxis(axisIndex) * scale;
      return Math.abs(value) < 0.08 ? 0.0 : value;
    }

    private boolean isPressed(GenericHID hid) {
      return Math.abs(read(hid)) >= pressThreshold;
    }
  }

  private record ButtonSpec(int buttonIndex) {
    private boolean isPressed(GenericHID hid) {
      return hid.getRawButton(buttonIndex);
    }
  }

  private record PovSpec(int angleDegrees) {
    private boolean isPressed(GenericHID hid) {
      return hid.getPOV() == angleDegrees;
    }
  }
}
