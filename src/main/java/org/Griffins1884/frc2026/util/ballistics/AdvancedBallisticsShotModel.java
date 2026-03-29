package org.Griffins1884.frc2026.util.ballistics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.OptionalDouble;

/**
 * Configurable moving-shot model with geometry offsets, compensation, and drag-aware simulation.
 */
public final class AdvancedBallisticsShotModel implements ShotModel {
  private static final double EPSILON = 1e-9;
  private static final int EXIT_GEOMETRY_ITERATIONS = 6;

  private final ShotModelConfig config;

  public AdvancedBallisticsShotModel(ShotModelConfig config) {
    this.config = config != null ? config : ShotModelConfig.defaultConfig();
  }

  public ShotModelConfig config() {
    return config;
  }

  @Override
  public ShotPrediction predict(ShotScenario scenario, LaunchCommand launchCommand) {
    Translation3d targetPosition =
        scenario != null && scenario.targetPositionMeters() != null
            ? scenario.targetPositionMeters()
            : new Translation3d();
    Translation2d robotVelocity =
        scenario != null && scenario.robotVelocityMetersPerSecond() != null
            ? scenario.robotVelocityMetersPerSecond()
            : new Translation2d();
    ShotModel.EntryWindow entryWindow =
        scenario != null ? scenario.entryWindow() : ShotModel.EntryWindow.unconstrained();
    ShotModel.ClearanceConstraint clearanceConstraint =
        scenario != null
            ? scenario.clearanceConstraint()
            : ShotModel.ClearanceConstraint.unconstrained();
    double clampedWheelRpm = config.flywheel().clampWheelRpm(launchCommand.wheelRpm());
    double clampedAngleDegrees =
        config.pivotCalibration().clampLaunchAngleDegrees(launchCommand.launchAngleDegrees());
    double launchMotorRotations =
        config.pivotCalibration().launchAngleDegreesToMotorRotations(clampedAngleDegrees);
    double exitVelocity = config.flywheel().rpmToExitVelocity(clampedWheelRpm);
    double launchAngleRadians = Math.toRadians(clampedAngleDegrees);
    double relativeHorizontalSpeed = exitVelocity * Math.cos(launchAngleRadians);
    double initialVerticalSpeed = exitVelocity * Math.sin(launchAngleRadians);
    Rotation2d turretYaw = new Rotation2d();
    Translation3d exitPosition = config.shooterExitPositionMeters(launchMotorRotations, turretYaw);
    Translation3d relativeTarget = targetPosition.minus(exitPosition);
    Translation2d targetHorizontal =
        new Translation2d(relativeTarget.getX(), relativeTarget.getY());
    double horizontalDistance = targetHorizontal.getNorm();

    if (relativeHorizontalSpeed <= EPSILON) {
      return new ShotPrediction(
          false,
          0.0,
          clampedAngleDegrees,
          launchMotorRotations,
          clampedWheelRpm,
          exitVelocity,
          0.0,
          horizontalDistance,
          Double.POSITIVE_INFINITY,
          Double.NEGATIVE_INFINITY,
          exitPosition,
          targetPosition,
          0.0,
          0.0);
    }

    InterceptSolution intercept =
        solveWithMovingExitGeometry(
            targetPosition,
            robotVelocity,
            relativeHorizontalSpeed,
            launchMotorRotations,
            exitPosition,
            horizontalDistance,
            relativeTarget);
    exitPosition = intercept.exitPosition();
    relativeTarget = intercept.relativeTarget();
    targetHorizontal = intercept.targetHorizontal();
    horizontalDistance = intercept.horizontalDistance();
    double[] robotMotionComponents = computeRobotMotionComponents(robotVelocity, targetHorizontal);

    if (!intercept.hasValidHorizontalSolution()) {
      return new ShotPrediction(
          false,
          0.0,
          clampedAngleDegrees,
          launchMotorRotations,
          clampedWheelRpm,
          exitVelocity,
          intercept.turretYaw().getDegrees(),
          horizontalDistance,
          Double.POSITIVE_INFINITY,
          Double.NEGATIVE_INFINITY,
          exitPosition,
          targetPosition,
          robotMotionComponents[0],
          robotMotionComponents[1]);
    }

    Translation2d relativeHorizontalVelocity = intercept.relativeHorizontalVelocity();
    Translation2d fieldHorizontalVelocity = relativeHorizontalVelocity.plus(robotVelocity);
    double turretYawDegrees = intercept.turretYaw().getDegrees();

    Translation3d initialVelocity =
        new Translation3d(
            fieldHorizontalVelocity.getX(), fieldHorizontalVelocity.getY(), initialVerticalSpeed);
    TrajectoryAssessment assessment =
        assessTrajectory(
            exitPosition, initialVelocity, targetPosition, entryWindow, clearanceConstraint);

    return new ShotPrediction(
        assessment.feasible(),
        assessment.referenceTimeSeconds(),
        clampedAngleDegrees,
        launchMotorRotations,
        clampedWheelRpm,
        exitVelocity,
        turretYawDegrees,
        horizontalDistance,
        assessment.errorMeters(),
        assessment.clearanceMarginMeters(),
        assessment.referencePositionMeters(),
        targetPosition,
        robotMotionComponents[0],
        robotMotionComponents[1]);
  }

  @Override
  public ShotSolution solve(ShotScenario scenario) {
    List<ShotCandidate> candidates = new ArrayList<>();
    int evaluations = 0;

    for (double angle = config.pivotCalibration().minLaunchAngleDegrees();
        angle <= config.pivotCalibration().maxLaunchAngleDegrees() + EPSILON;
        angle += config.search().launchAngleStepDegrees()) {
      for (double rpm = config.flywheel().minWheelRpm();
          rpm <= config.flywheel().maxWheelRpm() + EPSILON;
          rpm += config.search().rpmStep()) {
        ShotModel.LaunchCommand launchCommand = new ShotModel.LaunchCommand(rpm, angle);
        ShotPrediction prediction = predict(scenario, launchCommand);
        evaluations++;
        if (!Double.isFinite(prediction.closestApproachErrorMeters())) {
          continue;
        }
        candidates.add(new ShotCandidate(launchCommand, prediction));
      }
    }

    ShotCandidate best =
        candidates.stream()
            .min(
                Comparator.comparing(
                        (ShotCandidate candidate) -> !candidate.prediction().feasible())
                    .thenComparingDouble(
                        candidate -> candidate.prediction().closestApproachErrorMeters())
                    .thenComparingDouble(
                        candidate -> -candidate.prediction().clearanceMarginMeters())
                    .thenComparingDouble(candidate -> -candidate.prediction().launchAngleDegrees())
                    .thenComparingDouble(candidate -> candidate.prediction().timeOfFlightSeconds()))
            .orElseGet(
                () ->
                    new ShotCandidate(
                        new ShotModel.LaunchCommand(
                            config.flywheel().minWheelRpm(),
                            config.pivotCalibration().minLaunchAngleDegrees()),
                        predict(
                            scenario,
                            new ShotModel.LaunchCommand(
                                config.flywheel().minWheelRpm(),
                                config.pivotCalibration().minLaunchAngleDegrees()))));

    return new ShotSolution(best.launchCommand(), best.prediction(), evaluations);
  }

  @Override
  public String name() {
    return "advanced-ballistics-shot-model";
  }

  private static boolean shouldPreferEntry(EntryPoint candidate, EntryPoint currentBest) {
    if (currentBest == null) {
      return true;
    }
    if (candidate.valid() != currentBest.valid()) {
      return candidate.valid();
    }
    if (candidate.descending() != currentBest.descending()) {
      return candidate.descending();
    }
    if (Math.abs(candidate.horizontalErrorMeters() - currentBest.horizontalErrorMeters())
        > EPSILON) {
      return candidate.horizontalErrorMeters() < currentBest.horizontalErrorMeters();
    }
    return candidate.timeSeconds() < currentBest.timeSeconds();
  }

  private InterceptSolution solveWithMovingExitGeometry(
      Translation3d targetPosition,
      Translation2d robotVelocity,
      double relativeHorizontalSpeed,
      double launchMotorRotations,
      Translation3d initialExitPosition,
      double initialHorizontalDistance,
      Translation3d initialRelativeTarget) {
    Rotation2d turretYaw = new Rotation2d();
    Translation3d exitPosition = initialExitPosition;
    Translation3d relativeTarget = initialRelativeTarget;
    Translation2d targetHorizontal =
        new Translation2d(relativeTarget.getX(), relativeTarget.getY());
    double horizontalDistance = initialHorizontalDistance;
    Translation2d relativeHorizontalVelocity = new Translation2d();
    OptionalDouble interceptTimeOpt = OptionalDouble.empty();

    for (int iteration = 0; iteration < EXIT_GEOMETRY_ITERATIONS; iteration++) {
      targetHorizontal = new Translation2d(relativeTarget.getX(), relativeTarget.getY());
      horizontalDistance = targetHorizontal.getNorm();
      if (horizontalDistance <= EPSILON || relativeTarget.getZ() < -1.0) {
        return new InterceptSolution(
            false,
            turretYaw,
            exitPosition,
            relativeTarget,
            targetHorizontal,
            horizontalDistance,
            relativeHorizontalVelocity);
      }

      interceptTimeOpt =
          solveHorizontalInterceptTime(targetHorizontal, robotVelocity, relativeHorizontalSpeed);
      if (interceptTimeOpt.isEmpty()) {
        return new InterceptSolution(
            false,
            turretYaw,
            exitPosition,
            relativeTarget,
            targetHorizontal,
            horizontalDistance,
            relativeHorizontalVelocity);
      }

      double interceptTime = interceptTimeOpt.getAsDouble();
      Translation2d fieldHorizontalVelocity = targetHorizontal.times(1.0 / interceptTime);
      relativeHorizontalVelocity = fieldHorizontalVelocity.minus(robotVelocity);
      turretYaw =
          new Rotation2d(relativeHorizontalVelocity.getX(), relativeHorizontalVelocity.getY());

      Translation3d updatedExitPosition =
          config.shooterExitPositionMeters(launchMotorRotations, turretYaw);
      if (updatedExitPosition.getDistance(exitPosition) <= EPSILON) {
        exitPosition = updatedExitPosition;
        relativeTarget = targetPosition.minus(exitPosition);
        targetHorizontal = new Translation2d(relativeTarget.getX(), relativeTarget.getY());
        horizontalDistance = targetHorizontal.getNorm();
        return new InterceptSolution(
            true,
            turretYaw,
            exitPosition,
            relativeTarget,
            targetHorizontal,
            horizontalDistance,
            relativeHorizontalVelocity);
      }

      exitPosition = updatedExitPosition;
      relativeTarget = targetPosition.minus(exitPosition);
    }

    targetHorizontal = new Translation2d(relativeTarget.getX(), relativeTarget.getY());
    horizontalDistance = targetHorizontal.getNorm();
    return new InterceptSolution(
        interceptTimeOpt.isPresent(),
        turretYaw,
        exitPosition,
        relativeTarget,
        targetHorizontal,
        horizontalDistance,
        relativeHorizontalVelocity);
  }

  private OptionalDouble solveHorizontalInterceptTime(
      Translation2d targetHorizontal, Translation2d robotVelocity, double relativeHorizontalSpeed) {
    double a = dot(targetHorizontal, targetHorizontal);
    double b = -2.0 * dot(targetHorizontal, robotVelocity);
    double c =
        dot(robotVelocity, robotVelocity) - (relativeHorizontalSpeed * relativeHorizontalSpeed);

    List<Double> candidateTimes = new ArrayList<>();
    if (Math.abs(c) < EPSILON) {
      if (Math.abs(b) > EPSILON) {
        double t = -a / b;
        if (t > EPSILON) {
          candidateTimes.add(t);
        }
      }
    } else {
      double discriminant = (b * b) - (4.0 * c * a);
      if (discriminant < 0.0) {
        return OptionalDouble.empty();
      }
      double sqrt = Math.sqrt(discriminant);
      double t1 = (-b + sqrt) / (2.0 * c);
      double t2 = (-b - sqrt) / (2.0 * c);
      if (t1 > EPSILON) {
        candidateTimes.add(t1);
      }
      if (t2 > EPSILON) {
        candidateTimes.add(t2);
      }
    }

    return candidateTimes.stream()
        .filter(Double::isFinite)
        .filter(t -> t > EPSILON)
        .min(Double::compareTo)
        .stream()
        .mapToDouble(Double::doubleValue)
        .findFirst();
  }

  private TrajectoryAssessment assessTrajectory(
      Translation3d initialPosition,
      Translation3d initialVelocity,
      Translation3d targetPosition,
      ShotModel.EntryWindow entryWindow,
      ShotModel.ClearanceConstraint clearanceConstraint) {
    double dt = config.search().integrationStepSeconds();
    double maxTime = config.search().maxSimulationTimeSeconds();
    State state = new State(initialPosition, initialVelocity);
    TrajectoryPoint best =
        new TrajectoryPoint(0.0, initialPosition, initialPosition.getDistance(targetPosition));
    double targetZ = targetPosition.getZ();
    double entryRadius = entryWindow.horizontalRadiusMeters();
    double preferredUpperEntryZ = entryWindow.preferredUpperEntryHeightMeters();
    double preferredUpperEntryRadius = entryWindow.preferredUpperEntryRadiusMeters();
    boolean unconstrainedEntry = !Double.isFinite(entryRadius);
    boolean requireDescending = entryWindow.requireDescendingAtEntry();
    boolean requirePreferredUpperEntry =
        entryWindow.requirePreferredUpperEntry() && Double.isFinite(preferredUpperEntryZ);
    boolean unconstrainedClearance =
        (clearanceConstraint.startRadiusMeters() <= EPSILON
                && clearanceConstraint.endRadiusMeters() <= EPSILON)
            || (!Double.isFinite(clearanceConstraint.startRadiusMeters())
                && !Double.isFinite(clearanceConstraint.endRadiusMeters()));
    boolean clearanceViolated = false;
    double minClearanceMeters = Double.POSITIVE_INFINITY;
    EntryPoint bestEntry = null;
    EntryPoint bestUpperEntry = null;
    State previousState = state;
    double previousTime = 0.0;

    for (double time = 0.0; time <= maxTime + EPSILON; time += dt) {
      double error = state.position().getDistance(targetPosition);
      if (error < best.errorMeters()) {
        best = new TrajectoryPoint(time, state.position(), error);
      }
      if (!unconstrainedClearance) {
        double clearance = clearanceToEnvelope(state.position(), clearanceConstraint);
        minClearanceMeters = Math.min(minClearanceMeters, clearance);
        if (clearance < 0.0) {
          clearanceViolated = true;
        }
      }

      if (!unconstrainedEntry && time > 0.0) {
        boolean crossedTargetPlaneDescending =
            previousState.position().getZ() >= targetZ && state.position().getZ() <= targetZ;
        if (crossedTargetPlaneDescending) {
          double zDelta = previousState.position().getZ() - state.position().getZ();
          double interpolation =
              Math.abs(zDelta) <= EPSILON
                  ? 0.0
                  : (previousState.position().getZ() - targetZ) / zDelta;
          interpolation = clamp01(interpolation);
          Translation3d entryPosition =
              interpolate(previousState.position(), state.position(), interpolation);
          Translation3d entryVelocity =
              interpolate(previousState.velocity(), state.velocity(), interpolation);
          double entryTime = previousTime + ((time - previousTime) * interpolation);
          double horizontalError =
              entryPosition.toTranslation2d().getDistance(targetPosition.toTranslation2d());
          boolean descending = entryVelocity.getZ() < 0.0;
          boolean validEntry = horizontalError <= entryRadius && (!requireDescending || descending);
          EntryPoint candidate =
              new EntryPoint(entryTime, entryPosition, horizontalError, validEntry, descending);
          if (shouldPreferEntry(candidate, bestEntry)) {
            bestEntry = candidate;
          }
        }
      }
      if (requirePreferredUpperEntry && time > 0.0) {
        boolean crossedUpperPlaneDescending =
            previousState.position().getZ() >= preferredUpperEntryZ
                && state.position().getZ() <= preferredUpperEntryZ;
        if (crossedUpperPlaneDescending) {
          double zDelta = previousState.position().getZ() - state.position().getZ();
          double interpolation =
              Math.abs(zDelta) <= EPSILON
                  ? 0.0
                  : (previousState.position().getZ() - preferredUpperEntryZ) / zDelta;
          interpolation = clamp01(interpolation);
          Translation3d upperEntryPosition =
              interpolate(previousState.position(), state.position(), interpolation);
          Translation3d upperEntryVelocity =
              interpolate(previousState.velocity(), state.velocity(), interpolation);
          double upperEntryTime = previousTime + ((time - previousTime) * interpolation);
          double horizontalError =
              upperEntryPosition.toTranslation2d().getDistance(targetPosition.toTranslation2d());
          boolean descending = upperEntryVelocity.getZ() < 0.0;
          boolean validUpperEntry = horizontalError <= preferredUpperEntryRadius && descending;
          EntryPoint candidate =
              new EntryPoint(
                  upperEntryTime, upperEntryPosition, horizontalError, validUpperEntry, descending);
          if (shouldPreferEntry(candidate, bestUpperEntry)) {
            bestUpperEntry = candidate;
          }
        }
      }
      if (state.position().getZ() < 0.0 && state.velocity().getZ() <= 0.0 && time > 0.0) {
        break;
      }
      previousState = state;
      previousTime = time;
      state = integrate(state, dt);
    }

    if (unconstrainedEntry) {
      double clearancePenalty = clearanceViolated ? Math.abs(minClearanceMeters) : 0.0;
      double errorMeters = best.errorMeters() + clearancePenalty;
      boolean feasible =
          !clearanceViolated && best.errorMeters() <= config.search().solutionToleranceMeters();
      return new TrajectoryAssessment(
          feasible,
          best.timeSeconds(),
          errorMeters,
          best.positionMeters(),
          minClearanceMeters,
          best,
          bestEntry);
    }

    double entryPenalty =
        bestEntry != null
            ? Math.max(0.0, bestEntry.horizontalErrorMeters() - entryRadius)
            : best.errorMeters();
    if (bestEntry == null) {
      entryPenalty = Math.max(entryPenalty, best.errorMeters());
    }
    double preferredUpperEntryPenalty = 0.0;
    if (requirePreferredUpperEntry) {
      preferredUpperEntryPenalty =
          bestUpperEntry != null
              ? Math.max(0.0, bestUpperEntry.horizontalErrorMeters() - preferredUpperEntryRadius)
              : best.errorMeters();
      if (bestUpperEntry == null) {
        preferredUpperEntryPenalty = Math.max(preferredUpperEntryPenalty, best.errorMeters());
      }
    }
    double clearancePenalty = clearanceViolated ? Math.abs(minClearanceMeters) : 0.0;
    double errorMeters = preferredUpperEntryPenalty + entryPenalty + clearancePenalty;
    boolean feasible =
        bestEntry != null
            && bestEntry.valid()
            && (!requirePreferredUpperEntry || (bestUpperEntry != null && bestUpperEntry.valid()))
            && !clearanceViolated;
    double referenceTime = bestEntry != null ? bestEntry.timeSeconds() : best.timeSeconds();
    Translation3d referencePosition =
        bestEntry != null ? bestEntry.positionMeters() : best.positionMeters();
    return new TrajectoryAssessment(
        feasible,
        referenceTime,
        errorMeters,
        referencePosition,
        minClearanceMeters,
        best,
        bestEntry);
  }

  private State integrate(State state, double dt) {
    Derivative k1 = derivative(state);
    Derivative k2 = derivative(apply(state, k1, dt * 0.5));
    Derivative k3 = derivative(apply(state, k2, dt * 0.5));
    Derivative k4 = derivative(apply(state, k3, dt));

    Translation3d positionDelta =
        scale(
            add(
                add(k1.positionRate(), scale(add(k2.positionRate(), k3.positionRate()), 2.0)),
                k4.positionRate()),
            dt / 6.0);
    Translation3d velocityDelta =
        scale(
            add(
                add(k1.velocityRate(), scale(add(k2.velocityRate(), k3.velocityRate()), 2.0)),
                k4.velocityRate()),
            dt / 6.0);

    return new State(add(state.position(), positionDelta), add(state.velocity(), velocityDelta));
  }

  private Derivative derivative(State state) {
    return new Derivative(state.velocity(), accelerationFor(state.velocity()));
  }

  private State apply(State state, Derivative derivative, double scale) {
    return new State(
        add(state.position(), scale(derivative.positionRate(), scale)),
        add(state.velocity(), scale(derivative.velocityRate(), scale)));
  }

  private Translation3d accelerationFor(Translation3d velocity) {
    double speed = velocity.getNorm();
    ShotModelConfig.PhysicsConfig physics = config.physics();
    Translation3d drag =
        add(
            scale(velocity, -physics.linearDragPerSecond()),
            scale(velocity, -physics.quadraticDragPerMeter() * speed));
    Translation3d gravity = new Translation3d(0.0, 0.0, -physics.gravityMetersPerSecondSquared());
    return add(drag, gravity);
  }

  private static double dot(Translation2d a, Translation2d b) {
    return (a.getX() * b.getX()) + (a.getY() * b.getY());
  }

  private static double[] computeRobotMotionComponents(
      Translation2d robotVelocity, Translation2d targetHorizontal) {
    double horizontalDistance = targetHorizontal.getNorm();
    if (horizontalDistance <= EPSILON) {
      return new double[] {0.0, 0.0};
    }

    Translation2d radialDirection = targetHorizontal.times(1.0 / horizontalDistance);
    Translation2d tangentialDirection =
        new Translation2d(-radialDirection.getY(), radialDirection.getX());
    return new double[] {
      dot(robotVelocity, radialDirection), dot(robotVelocity, tangentialDirection)
    };
  }

  private static Translation3d add(Translation3d a, Translation3d b) {
    return new Translation3d(a.getX() + b.getX(), a.getY() + b.getY(), a.getZ() + b.getZ());
  }

  private static Translation3d scale(Translation3d value, double scalar) {
    return new Translation3d(value.getX() * scalar, value.getY() * scalar, value.getZ() * scalar);
  }

  private static Translation3d interpolate(Translation3d a, Translation3d b, double t) {
    return new Translation3d(
        a.getX() + ((b.getX() - a.getX()) * t),
        a.getY() + ((b.getY() - a.getY()) * t),
        a.getZ() + ((b.getZ() - a.getZ()) * t));
  }

  private static double clearanceToEnvelope(
      Translation3d position, ShotModel.ClearanceConstraint constraint) {
    Translation3d start = constraint.startMeters();
    Translation3d end = constraint.endMeters();
    Translation3d axis = end.minus(start);
    double axisLengthSquared =
        (axis.getX() * axis.getX()) + (axis.getY() * axis.getY()) + (axis.getZ() * axis.getZ());

    if (axisLengthSquared <= EPSILON) {
      return position.getDistance(start) - constraint.startRadiusMeters();
    }

    Translation3d offset = position.minus(start);
    double projection =
        ((offset.getX() * axis.getX())
                + (offset.getY() * axis.getY())
                + (offset.getZ() * axis.getZ()))
            / axisLengthSquared;
    double t = clamp01(projection);
    Translation3d closestPoint = interpolate(start, end, t);
    double radius =
        constraint.startRadiusMeters()
            + ((constraint.endRadiusMeters() - constraint.startRadiusMeters()) * t);
    return position.getDistance(closestPoint) - radius;
  }

  private static double clamp01(double value) {
    return Math.max(0.0, Math.min(1.0, value));
  }

  private record State(Translation3d position, Translation3d velocity) {}

  private record Derivative(Translation3d positionRate, Translation3d velocityRate) {}

  private record TrajectoryPoint(
      double timeSeconds, Translation3d positionMeters, double errorMeters) {}

  private record EntryPoint(
      double timeSeconds,
      Translation3d positionMeters,
      double horizontalErrorMeters,
      boolean valid,
      boolean descending) {}

  private record TrajectoryAssessment(
      boolean feasible,
      double referenceTimeSeconds,
      double errorMeters,
      Translation3d referencePositionMeters,
      double clearanceMarginMeters,
      TrajectoryPoint closestApproach,
      EntryPoint bestEntry) {}

  private record InterceptSolution(
      boolean hasValidHorizontalSolution,
      Rotation2d turretYaw,
      Translation3d exitPosition,
      Translation3d relativeTarget,
      Translation2d targetHorizontal,
      double horizontalDistance,
      Translation2d relativeHorizontalVelocity) {}

  private record ShotCandidate(ShotModel.LaunchCommand launchCommand, ShotPrediction prediction) {}
}
