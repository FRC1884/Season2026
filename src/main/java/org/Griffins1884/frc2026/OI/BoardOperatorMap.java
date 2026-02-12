package org.Griffins1884.frc2026.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  // TODO: Update these placeholders to match the board wiring.
  private static final int AUTO_MANUAL_TOGGLE_BUTTON = 7;
  private static final int MANUAL_TURRET_AXIS = 0;
  private static final int MANUAL_PIVOT_AXIS = 1;

  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger testing() {
    return button(1);
  }

  @Override
  public Trigger turretZero() {
    return button(2);
  }

  @Override
  public Trigger turretPreset() {
    return button(3);
  }

  @Override
  public Trigger turretManualLeft() {
    return button(4);
  }

  @Override
  public Trigger turretManualRight() {
    return button(5);
  }

  @Override
  public Trigger turretAutoAim() {
    return button(6);
  }

  @Override
  public Trigger autoManualToggle() {
    return button(AUTO_MANUAL_TOGGLE_BUTTON);
  }

  @Override
  public DoubleSupplier manualTurretAxis() {
    return () -> getRawAxis(MANUAL_TURRET_AXIS);
  }

  @Override
  public DoubleSupplier manualPivotAxis() {
    return () -> getRawAxis(MANUAL_PIVOT_AXIS);
  }
}
