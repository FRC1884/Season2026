package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VelocityRollerMechanism;

@Getter
public class ShooterSubsystem extends VelocityRollerMechanism<ShooterSubsystem.ShooterGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterGoal implements VelocityGoal {
    IDLING(() -> 0.0);

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  private final ShooterGoal goal = ShooterGoal.IDLING;

  public ShooterSubsystem(String name, ShooterIO io) {
    super(
        name,
        RobotMechanismDefinitions.SHOOTER,
        io,
        new VelocityRollerConfig(
            ShooterConstants.GAINS,
            ShooterConstants.VELOCITY_TOLERANCE,
            ShooterConstants.MAX_VOLTAGE));
    if (supportsOnboardVelocityControl()) {
      configureOnboardVelocitySlot(0, ShooterConstants.GAINS);
    }
  }

  @Override
  protected double getAdditionalCompensationVolts(
      double goalVelocityRpm, double measuredVelocityRpm) {
    double goalSign = Math.signum(goalVelocityRpm);
    if (goalSign == 0.0) {
      return 0.0;
    }

    double velocityDeficitRpm =
        Math.max(0.0, Math.abs(goalVelocityRpm) - Math.abs(measuredVelocityRpm));
    if (velocityDeficitRpm <= 0.0) {
      return 0.0;
    }

    double errorCompensationVolts =
        velocityDeficitRpm * ShooterConstants.RECOVERY_ERROR_GAIN_VOLTS_PER_RPM.get();
    double currentOverThresholdAmps =
        Math.max(
            0.0, getSupplyCurrentAmps() - ShooterConstants.RECOVERY_CURRENT_THRESHOLD_AMPS.get());
    double currentCompensationVolts =
        currentOverThresholdAmps * ShooterConstants.RECOVERY_CURRENT_GAIN_VOLTS_PER_AMP.get();
    double totalBoostVolts =
        MathUtil.clamp(
            errorCompensationVolts + currentCompensationVolts,
            0.0,
            ShooterConstants.RECOVERY_MAX_BOOST_VOLTS.get());
    return goalSign * totalBoostVolts;
  }

  public void setTargetVelocityRpm(double velocityRpm) {
    setGoalVelocity(velocityRpm);
  }
}
