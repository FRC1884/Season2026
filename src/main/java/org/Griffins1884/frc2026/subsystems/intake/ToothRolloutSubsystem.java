package org.Griffins1884.frc2026.subsystems.intake;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VoltageRollerMechanism;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class ToothRolloutSubsystem
    extends VoltageRollerMechanism<ToothRolloutSubsystem.ToothRolloutGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ToothRolloutGoal implements VoltageGoal {
    IDLING(() -> 0.0),
    HOLD(() -> ToothRolloutConstants.HOLD_VOLTS.get()),
    DEPLOY(() -> ToothRolloutConstants.DEPLOY_VOLTS.get()),
    RETRACT(() -> ToothRolloutConstants.RETRACT_VOLTS.get()),
    TESTING(new LoggedTunableNumber("ToothRollout/TestingVolts", 0.0));

    private final DoubleSupplier voltageSupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return voltageSupplier;
    }
  }

  private ToothRolloutGoal goal = ToothRolloutGoal.IDLING;

  public ToothRolloutSubsystem(String name, ToothRolloutIO io) {
    super(
        name,
        RobotMechanismDefinitions.TOOTH_ROLLOUT,
        io,
        new VoltageRollerConfig(ToothRolloutConstants.MAX_VOLTAGE));
  }
}
