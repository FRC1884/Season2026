package org.Griffins1884.frc2026.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
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
}
