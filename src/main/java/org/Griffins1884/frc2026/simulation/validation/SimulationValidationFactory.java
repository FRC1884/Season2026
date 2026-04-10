package org.Griffins1884.frc2026.simulation.validation;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.simulation.drive.Season2026DriveSimulation;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.RenderTelemetry;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

/** Creates the requested unattended validation routine for the real desktop simulateJava path. */
public final class SimulationValidationFactory {
  private SimulationValidationFactory() {}

  public static SimulationValidationRoutine createIfEnabled(
      SwerveSubsystem drive,
      Superstructure superstructure,
      Season2026DriveSimulation terrainSimulation,
      Supplier<RenderTelemetry> renderTelemetrySupplier,
      Supplier<RobotStateVisualizer.ShotTelemetrySnapshot> shotTelemetrySupplier,
      Consumer<Pose2d> simulationReset) {
    if (!Boolean.getBoolean("season2026.simValidation.enabled")) {
      return null;
    }

    String mode = System.getProperty("season2026.simValidation.mode", "bump").trim();
    return switch (mode) {
      case "shot" ->
          ShotValidationController.create(
              drive,
              superstructure,
              renderTelemetrySupplier,
              shotTelemetrySupplier,
              simulationReset);
      case "bump", "" ->
          SimulationValidationController.create(
              drive, terrainSimulation, renderTelemetrySupplier, simulationReset);
      default ->
          throw new IllegalArgumentException(
              "Unsupported season2026.simValidation.mode='" + mode + "'");
    };
  }
}
