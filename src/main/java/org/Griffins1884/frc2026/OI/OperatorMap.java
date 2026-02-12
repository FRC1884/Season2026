package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }

  Trigger testing();

  Trigger turretZero();

  Trigger turretPreset();

  Trigger turretManualLeft();

  Trigger turretManualRight();

  Trigger turretAutoAim();

  default Trigger autoManualToggle() {
    return new Trigger(() -> false);
  }

  default DoubleSupplier manualTurretAxis() {
    return () -> 0.0;
  }

  default DoubleSupplier manualPivotAxis() {
    return () -> 0.0;
  }
}
