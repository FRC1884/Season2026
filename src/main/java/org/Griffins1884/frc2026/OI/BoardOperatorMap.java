package org.Griffins1884.frc2026.OI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  // TODO: Update these placeholders to match the board wiring.
  private static final int AUTO_MANUAL_TOGGLE_BUTTON = 6;
  private static final int MANUAL_TURRET_AXIS = 0;
  private static final int MANUAL_PIVOT_AXIS = 1;

  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger intake() {
    return safeButton(3);
  }

  @Override
  public Trigger shooter() {
    return safeButton(7);
  }

  @Override
  public Trigger endgameClimb() {
    return safeButton(4);
  }

  @Override
  public Trigger detachClimb() {
    return safeButton(5);
  }

  @Override
  public Trigger idling() {
    return safeButton(2);
  }

  @Override
  public Trigger ferrying() {
    return safeButton(1);
  }

  @Override
  public DoubleSupplier manualTurretAxis() {
    return () -> getRawAxis(MANUAL_TURRET_AXIS);
  }

  @Override
  public DoubleSupplier manualPivotAxis() {
    return () -> getRawAxis(MANUAL_PIVOT_AXIS);
  }

  @Override
  public Trigger autoManualToggle() {
    return safeButton(AUTO_MANUAL_TOGGLE_BUTTON);
  }

  private Trigger safeButton(int button) {
    return new Trigger(
        () -> {
          int port = getHID().getPort();
          if (!DriverStation.isJoystickConnected(port)) {
            return false;
          }
          if (button <= 0 || DriverStation.getStickButtonCount(port) < button) {
            return false;
          }
          return getHID().getRawButton(button);
        });
  }
}
