package org.Griffins1884.frc2026.util.ballistics;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Exports raw and solved shot sweeps for offline fitting or MCP/LLM analysis. */
public final class ShotSweepExporterMain {
  private static final ObjectMapper JSON =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
  private static final double DEFAULT_TARGET_HEIGHT_METERS = 56.5 * 0.0254;

  private ShotSweepExporterMain() {}

  public static void main(String... args) throws IOException {
    Map<String, String> options = parseArgs(args);
    Path outputDir =
        Paths.get(options.getOrDefault("--output-dir", "build/reports/ballistics"))
            .toAbsolutePath();
    Files.createDirectories(outputDir);

    ShotModelConfig config = ShotModelConfig.defaultConfig();
    AdvancedBallisticsShotModel model = new AdvancedBallisticsShotModel(config);
    SweepDefinition sweep = SweepDefinition.defaultSweep();

    List<GridSample> samples = new ArrayList<>();
    List<SolutionSample> bestSolutions = new ArrayList<>();

    for (double distanceMeters : sweep.distanceMeters().values()) {
      for (double lateralMeters : sweep.targetLateralMeters().values()) {
        Translation3d targetPosition =
            new Translation3d(distanceMeters, lateralMeters, DEFAULT_TARGET_HEIGHT_METERS);
        for (double forwardVelocity : sweep.robotForwardVelocityMetersPerSecond().values()) {
          for (double lateralVelocity : sweep.robotLateralVelocityMetersPerSecond().values()) {
            ShotModel.ShotScenario scenario =
                new ShotModel.ShotScenario(
                    targetPosition, new Translation2d(forwardVelocity, lateralVelocity));
            ShotModel.ShotSolution best = model.solve(scenario);
            bestSolutions.add(new SolutionSample(scenario, best));
            for (double angleDegrees : sweep.launchAngleDegrees().values()) {
              for (double wheelRpm : sweep.wheelRpm().values()) {
                ShotModel.LaunchCommand command =
                    new ShotModel.LaunchCommand(wheelRpm, angleDegrees);
                samples.add(new GridSample(scenario, command, model.predict(scenario, command)));
              }
            }
          }
        }
      }
    }

    SweepExport export =
        new SweepExport(
            Instant.now().toString(),
            model.name(),
            config,
            sweep,
            samples.size(),
            bestSolutions.size(),
            samples,
            bestSolutions);

    Path jsonPath = outputDir.resolve("shot-sweep.json");
    Path rawCsvPath = outputDir.resolve("shot-sweep.csv");
    Path bestCsvPath = outputDir.resolve("best-shot-solutions.csv");

    JSON.writeValue(jsonPath.toFile(), export);
    writeRawCsv(rawCsvPath, samples);
    writeBestCsv(bestCsvPath, bestSolutions);
  }

  private static void writeRawCsv(Path output, List<GridSample> samples) throws IOException {
    try (BufferedWriter writer = Files.newBufferedWriter(output, StandardCharsets.UTF_8)) {
      writer.write(
          "targetX,targetY,targetZ,robotVx,robotVy,wheelRpm,launchAngleDeg,launchMotorRotations,feasible,timeOfFlight,turretYawDeg,exitVelocity,horizontalDistance,error,closestX,closestY,closestZ,robotRadialVelocity,robotTangentialVelocity\n");
      for (GridSample sample : samples) {
        ShotModel.ShotScenario scenario = sample.scenario();
        ShotModel.LaunchCommand command = sample.command();
        ShotModel.ShotPrediction prediction = sample.prediction();
        writer.write(
            csv(
                scenario.targetPositionMeters().getX(),
                scenario.targetPositionMeters().getY(),
                scenario.targetPositionMeters().getZ(),
                scenario.robotVelocityMetersPerSecond().getX(),
                scenario.robotVelocityMetersPerSecond().getY(),
                command.wheelRpm(),
                prediction.launchAngleDegrees(),
                prediction.launchMotorRotations(),
                prediction.feasible(),
                prediction.timeOfFlightSeconds(),
                prediction.turretYawDegrees(),
                prediction.exitVelocityMetersPerSecond(),
                prediction.horizontalDistanceMeters(),
                prediction.closestApproachErrorMeters(),
                prediction.closestApproachPositionMeters().getX(),
                prediction.closestApproachPositionMeters().getY(),
                prediction.closestApproachPositionMeters().getZ(),
                prediction.robotRadialVelocityMetersPerSecond(),
                prediction.robotTangentialVelocityMetersPerSecond()));
        writer.newLine();
      }
    }
  }

  private static void writeBestCsv(Path output, List<SolutionSample> bestSolutions)
      throws IOException {
    try (BufferedWriter writer = Files.newBufferedWriter(output, StandardCharsets.UTF_8)) {
      writer.write(
          "targetX,targetY,targetZ,robotVx,robotVy,evaluations,bestWheelRpm,bestLaunchAngleDeg,bestMotorRotations,feasible,timeOfFlight,turretYawDeg,exitVelocity,error\n");
      for (SolutionSample sample : bestSolutions) {
        ShotModel.ShotScenario scenario = sample.scenario();
        ShotModel.ShotSolution solution = sample.solution();
        ShotModel.ShotPrediction prediction = solution.prediction();
        writer.write(
            csv(
                scenario.targetPositionMeters().getX(),
                scenario.targetPositionMeters().getY(),
                scenario.targetPositionMeters().getZ(),
                scenario.robotVelocityMetersPerSecond().getX(),
                scenario.robotVelocityMetersPerSecond().getY(),
                solution.evaluations(),
                solution.launchCommand().wheelRpm(),
                prediction.launchAngleDegrees(),
                prediction.launchMotorRotations(),
                prediction.feasible(),
                prediction.timeOfFlightSeconds(),
                prediction.turretYawDegrees(),
                prediction.exitVelocityMetersPerSecond(),
                prediction.closestApproachErrorMeters()));
        writer.newLine();
      }
    }
  }

  private static String csv(Object... values) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < values.length; i++) {
      if (i > 0) {
        builder.append(',');
      }
      builder.append(values[i]);
    }
    return builder.toString();
  }

  private static Map<String, String> parseArgs(String[] args) {
    Map<String, String> options = new LinkedHashMap<>();
    for (int i = 0; i < args.length; i++) {
      String arg = args[i];
      if (!arg.startsWith("--")) {
        continue;
      }
      if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
        options.put(arg, args[++i]);
      } else {
        options.put(arg, "true");
      }
    }
    return options;
  }

  record GridSample(
      ShotModel.ShotScenario scenario,
      ShotModel.LaunchCommand command,
      ShotModel.ShotPrediction prediction) {}

  record SolutionSample(ShotModel.ShotScenario scenario, ShotModel.ShotSolution solution) {}

  record SweepExport(
      String generatedAt,
      String modelName,
      ShotModelConfig config,
      SweepDefinition sweep,
      int rawSampleCount,
      int solvedScenarioCount,
      List<GridSample> rawSamples,
      List<SolutionSample> bestSolutions) {}

  record SweepDefinition(
      SweepAxis distanceMeters,
      SweepAxis targetLateralMeters,
      SweepAxis robotForwardVelocityMetersPerSecond,
      SweepAxis robotLateralVelocityMetersPerSecond,
      SweepAxis launchAngleDegrees,
      SweepAxis wheelRpm) {
    static SweepDefinition defaultSweep() {
      ShotModelConfig config = ShotModelConfig.defaultConfig();
      return new SweepDefinition(
          new SweepAxis(2.0, 8.0, 0.5),
          new SweepAxis(0.0, 0.0, 1.0),
          new SweepAxis(-2.0, 2.0, 1.0),
          new SweepAxis(-2.0, 2.0, 1.0),
          new SweepAxis(
              config.pivotCalibration().minLaunchAngleDegrees(),
              config.pivotCalibration().maxLaunchAngleDegrees(),
              3.0),
          new SweepAxis(config.flywheel().minWheelRpm(), config.flywheel().maxWheelRpm(), 250.0));
    }
  }

  record SweepAxis(double startInclusive, double endInclusive, double step) {
    List<Double> values() {
      List<Double> values = new ArrayList<>();
      if (step <= 0.0) {
        values.add(startInclusive);
        return values;
      }
      for (double value = startInclusive; value <= endInclusive + 1e-9; value += step) {
        values.add(Math.round(value * 1000.0) / 1000.0);
      }
      return values;
    }
  }
}
