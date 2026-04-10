package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

final class DeployAutoLibrary {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  private static final double ROUTE_POINT_SPACING_METERS = 1.1;
  private static final double STOPPED_END_VELOCITY_MPS = 0.05;

  private final Path pathPlannerAutosRoot;
  private String lastFailureMessage = "";

  DeployAutoLibrary(Path pathPlannerAutosRoot) {
    this.pathPlannerAutosRoot =
        Objects.requireNonNull(pathPlannerAutosRoot, "pathPlannerAutosRoot")
            .toAbsolutePath()
            .normalize();
  }

  Optional<LoadedAuto> loadAuto(String autoId) {
    lastFailureMessage = "";
    String normalizedId = blankToNull(autoId);
    if (normalizedId == null) {
      return Optional.empty();
    }
    return findManifestEntry(normalizedId).flatMap(this::loadAuto);
  }

  String getLastFailureMessage() {
    return lastFailureMessage;
  }

  String buildManifestJson() {
    try {
      return JSON.writeValueAsString(
          new LibraryIndexDto("2026.0", "DeployAutoLibrary", listAutos()));
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize deploy auto manifest", ex.getStackTrace());
      return "{\"autos\":[]}";
    }
  }

  Optional<String> loadAutoPreviewJson(String relativePath) {
    String normalized = blankToNull(relativePath);
    if (normalized == null) {
      return Optional.empty();
    }
    String autoId =
        normalized.toLowerCase(Locale.ROOT).endsWith(".json")
            ? normalized.substring(0, normalized.length() - 5)
            : normalized;
    Optional<LoadedAuto> loadedAuto = loadAuto(autoId);
    if (loadedAuto.isEmpty()) {
      return Optional.empty();
    }
    try {
      return Optional.of(JSON.writeValueAsString(loadedAuto.get()));
    } catch (JsonProcessingException ex) {
      DriverStation.reportError(
          "Failed to serialize deploy auto preview \"" + normalized + "\"", ex.getStackTrace());
      return Optional.empty();
    }
  }

  private List<AutoManifestEntryDto> listAutos() {
    if (!Files.isDirectory(pathPlannerAutosRoot)) {
      return List.of();
    }
    try (var files = Files.walk(pathPlannerAutosRoot)) {
      return files
          .filter(Files::isRegularFile)
          .filter(path -> path.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".auto"))
          .sorted(
              Comparator.comparing(path -> path.getFileName().toString().toLowerCase(Locale.ROOT)))
          .map(this::toManifestEntry)
          .filter(Objects::nonNull)
          .toList();
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to enumerate PathPlanner autos: " + ex.getMessage(), ex.getStackTrace());
      return List.of();
    }
  }

  private AutoManifestEntryDto toManifestEntry(Path autoFile) {
    String autoId = autoName(autoFile);
    if (autoId == null) {
      return null;
    }
    RawAutoDto raw = readRawAuto(autoFile).orElse(null);
    String folder = raw != null ? blankToNull(raw.folder) : null;
    if (folder == null) {
      Path parent = autoFile.getParent();
      if (parent != null && !parent.equals(pathPlannerAutosRoot)) {
        folder = blankToNull(pathPlannerAutosRoot.relativize(parent).toString().replace('\\', '/'));
      }
    }
    long updatedAt = 0L;
    try {
      updatedAt = Files.getLastModifiedTime(autoFile).toMillis();
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Failed to read modified time for PathPlanner auto \"" + autoFile + "\".", false);
    }
    return new AutoManifestEntryDto(autoId, autoId, folder, autoId + ".json", updatedAt);
  }

  private Optional<AutoManifestEntryDto> findManifestEntry(String autoId) {
    return listAutos().stream().filter(entry -> autoId.equals(entry.id)).findFirst();
  }

  private Optional<LoadedAuto> loadAuto(AutoManifestEntryDto entry) {
    Path autoFile = pathPlannerAutosRoot.resolve(entry.id + ".auto").normalize().toAbsolutePath();
    if (!autoFile.startsWith(pathPlannerAutosRoot) || !Files.isRegularFile(autoFile)) {
      return Optional.empty();
    }
    try {
      JSONObject autoJson = readJsonObject(autoFile);
      boolean choreoAuto = Boolean.TRUE.equals(autoJson.get("choreoAuto"));
      JSONObject commandJson = requireObject(autoJson.get("command"), entry.id + " command");

      ConvertedAuto converted = convertAutoCommands(entry.id, commandJson, choreoAuto);
      return Optional.of(
          new LoadedAuto(
              entry.id,
              entry.name,
              entry.folder,
              entry.relativePath,
              entry.updatedAt,
              converted.startPose(),
              List.of(),
              converted.steps()));
    } catch (LoadFailure ex) {
      lastFailureMessage = ex.getMessage();
      DriverStation.reportError(lastFailureMessage, ex.getStackTrace());
      return Optional.empty();
    } catch (IOException | ParseException ex) {
      lastFailureMessage =
          "Failed to load PathPlanner auto \"" + entry.id + "\": " + ex.getMessage();
      DriverStation.reportError(lastFailureMessage, ex.getStackTrace());
      return Optional.empty();
    }
  }

  private ConvertedAuto convertAutoCommands(
      String autoId, JSONObject commandJson, boolean choreoAuto)
      throws IOException, ParseException, LoadFailure {
    ArrayList<StepSpec> steps = new ArrayList<>();
    ConversionContext context = new ConversionContext();
    flattenCommand(autoId, commandJson, choreoAuto, steps, context);
    return new ConvertedAuto(context.startPose, List.copyOf(steps));
  }

  private void flattenCommand(
      String autoId,
      JSONObject commandJson,
      boolean choreoAuto,
      ArrayList<StepSpec> steps,
      ConversionContext context)
      throws IOException, ParseException, LoadFailure {
    String type = stringValue(commandJson.get("type"));
    JSONObject data = requireObject(commandJson.get("data"), autoId + " command data");
    switch (type) {
      case "sequential" -> {
        JSONArray commands =
            requireArray(data.get("commands"), autoId + " sequential command list");
        for (Object child : commands) {
          flattenCommand(
              autoId,
              requireObject(child, autoId + " sequential child command"),
              choreoAuto,
              steps,
              context);
        }
      }
      case "path" -> {
        String pathName = requireString(data.get("pathName"), autoId + " path command");
        ConvertedPath convertedPath = convertPath(autoId, pathName, choreoAuto);
        if (context.startPose == null) {
          context.startPose = convertedPath.startPose();
        }
        steps.addAll(convertedPath.steps());
        context.currentBoundaryPose = convertedPath.endPose();
      }
      case "named" -> {
        String commandName = requireNamedCommandName(data, autoId + " named command");
        steps.add(commandStep(commandName, commandName, context.currentBoundaryPose));
      }
      case "wait" -> {
        double waitSeconds = requireWaitSeconds(data, autoId + " wait command");
        steps.add(waitStep(waitSeconds, context.currentBoundaryPose));
      }
      case "parallel", "race", "deadline" ->
          throw new LoadFailure(
              "Deploy auto \""
                  + autoId
                  + "\" uses unsupported PathPlanner command type \""
                  + type
                  + "\". Queue execution only supports sequential path, named, and wait commands.");
      default ->
          throw new LoadFailure(
              "Deploy auto \""
                  + autoId
                  + "\" contains unknown PathPlanner command type \""
                  + type
                  + "\".");
    }
  }

  private ConvertedPath convertPath(String autoId, String pathName, boolean choreoAuto)
      throws IOException, ParseException, LoadFailure {
    PathPlannerPath path =
        choreoAuto
            ? PathPlannerPath.fromChoreoTrajectory(pathName)
            : PathPlannerPath.fromPathFile(pathName);
    List<PathSample> samples = buildPathSamples(path, pathName);
    if (samples.isEmpty()) {
      throw new LoadFailure(
          "Path \"" + pathName + "\" in auto \"" + autoId + "\" has no usable poses.");
    }

    ArrayList<PathSample> boundaries = selectRouteBoundaries(samples);
    ArrayList<MarkerInsertion> markers =
        new ArrayList<>(loadPathMarkers(pathName, choreoAuto, boundaries, path));

    PoseSpec startPose = poseSpec(samples.get(0).pose());
    PoseSpec endPose = poseSpec(samples.get(samples.size() - 1).pose());

    double pathMaxVelocityMps =
        samples.stream()
            .mapToDouble(PathSample::maxVelocityMps)
            .filter(Double::isFinite)
            .max()
            .orElse(Double.NaN);
    double pathEndVelocityMps =
        path.getGoalEndState() != null ? path.getGoalEndState().velocityMPS() : 0.0;

    ArrayList<StepSpec> steps = new ArrayList<>();
    ArrayList<Integer> markerBoundaryIndexes =
        new ArrayList<>(
            markers.stream().map(MarkerInsertion::boundaryIndex).distinct().sorted().toList());

    int segmentStartBoundary = 0;
    int segmentOrdinal = 1;
    int segmentTotal = markerBoundaryIndexes.size() + 1;
    for (int markerBoundaryIndex : markerBoundaryIndexes) {
      if (markerBoundaryIndex < 0 || markerBoundaryIndex >= boundaries.size()) {
        continue;
      }
      PathSegmentStep segment =
          buildPathSegmentStep(
              pathName,
              segmentOrdinal++,
              segmentTotal,
              boundaries,
              segmentStartBoundary,
              markerBoundaryIndex,
              pathMaxVelocityMps,
              boundaryEndVelocityMps(boundaries.get(markerBoundaryIndex), pathMaxVelocityMps),
              false);
      if (segment != null) {
        steps.add(segment.step());
      }
      for (MarkerInsertion marker : markers) {
        if (marker.boundaryIndex() == markerBoundaryIndex) {
          steps.add(
              commandStep(
                  marker.commandName(),
                  marker.commandName(),
                  poseSpec(boundaries.get(markerBoundaryIndex).pose())));
        }
      }
      segmentStartBoundary = markerBoundaryIndex + 1;
    }

    PathSegmentStep finalSegment =
        buildPathSegmentStep(
            pathName,
            segmentOrdinal,
            segmentTotal,
            boundaries,
            segmentStartBoundary,
            boundaries.size() - 1,
            pathMaxVelocityMps,
            pathEndVelocityMps,
            isStopped(pathEndVelocityMps));
    if (finalSegment != null) {
      steps.add(finalSegment.step());
    }

    return new ConvertedPath(startPose, endPose, List.copyOf(steps));
  }

  private static ArrayList<PathSample> buildPathSamples(PathPlannerPath path, String pathName)
      throws LoadFailure {
    ArrayList<PathSample> samples = new ArrayList<>();
    if (path.isChoreoPath()) {
      PathPlannerTrajectory trajectory = path.getIdealTrajectory(null).orElse(null);
      if (trajectory == null || trajectory.getStates().isEmpty()) {
        throw new LoadFailure(
            "Choreo path \"" + pathName + "\" did not provide trajectory states.");
      }
      double distanceMeters = 0.0;
      Pose2d previousPose = null;
      for (PathPlannerTrajectoryState state : trajectory.getStates()) {
        if (previousPose != null) {
          distanceMeters += previousPose.getTranslation().getDistance(state.pose.getTranslation());
        }
        samples.add(
            new PathSample(
                state.pose,
                distanceMeters,
                Double.NaN,
                Math.max(0.0, state.linearVelocity),
                state.timeSeconds));
        previousPose = state.pose;
      }
      return samples;
    }

    List<PathPoint> points = path.getAllPathPoints();
    if (points.isEmpty()) {
      return samples;
    }
    GoalEndState goalEndState = path.getGoalEndState();
    Rotation2d goalRotation = goalEndState != null ? goalEndState.rotation() : Rotation2d.kZero;
    for (int i = 0; i < points.size(); i++) {
      PathPoint point = points.get(i);
      samples.add(
          new PathSample(
              new Pose2d(point.position, headingForPoint(points, i, goalRotation)),
              point.distanceAlongPath,
              point.waypointRelativePos,
              point.maxV,
              Double.NaN));
    }
    return samples;
  }

  private static ArrayList<PathSample> selectRouteBoundaries(List<PathSample> samples) {
    ArrayList<PathSample> boundaries = new ArrayList<>();
    if (samples.isEmpty()) {
      return boundaries;
    }
    double lastDistanceMeters = 0.0;
    for (int i = 1; i < samples.size() - 1; i++) {
      PathSample sample = samples.get(i);
      if (sample.distanceMeters() - lastDistanceMeters >= ROUTE_POINT_SPACING_METERS) {
        boundaries.add(sample);
        lastDistanceMeters = sample.distanceMeters();
      }
    }
    boundaries.add(samples.get(samples.size() - 1));
    return boundaries;
  }

  private List<MarkerInsertion> loadPathMarkers(
      String pathName, boolean choreoAuto, List<PathSample> boundaries, PathPlannerPath path)
      throws IOException, ParseException, LoadFailure {
    if (boundaries.isEmpty()) {
      return List.of();
    }
    JSONObject json = choreoAuto ? readChoreoJson(pathName) : readPathJson(pathName);
    ArrayList<MarkerInsertion> insertions = new ArrayList<>();
    if (choreoAuto) {
      JSONArray events = (JSONArray) json.getOrDefault("events", new JSONArray());
      for (Object rawEvent : events) {
        JSONObject eventJson = requireObject(rawEvent, pathName + " choreo event");
        JSONObject commandJson = objectValue(eventJson.get("event"));
        if (commandJson == null) {
          continue;
        }
        ResolvedCommand command = resolveBoundaryCommand(commandJson, pathName + " choreo event");
        if (!command.isNamedCommand()) {
          continue;
        }
        JSONObject fromJson =
            requireObject(eventJson.get("from"), pathName + " choreo event timing");
        double timestampSeconds =
            numericValue(fromJson.get("targetTimestamp"))
                + nestedNumericValue(fromJson, "offset", "val");
        int boundaryIndex = nearestBoundaryByTime(boundaries, timestampSeconds);
        insertions.add(new MarkerInsertion(boundaryIndex, command.commandName()));
      }
      return List.copyOf(insertions);
    }

    JSONArray markers = (JSONArray) json.getOrDefault("eventMarkers", new JSONArray());
    for (Object rawMarker : markers) {
      JSONObject markerJson = requireObject(rawMarker, pathName + " path marker");
      JSONObject commandJson = objectValue(markerJson.get("command"));
      if (commandJson == null) {
        continue;
      }
      ResolvedCommand command = resolveBoundaryCommand(commandJson, pathName + " path marker");
      if (!command.isNamedCommand()) {
        continue;
      }
      double progress = numericValue(markerJson.get("waypointRelativePos"));
      int boundaryIndex = nearestBoundaryByProgress(boundaries, progress, path);
      insertions.add(new MarkerInsertion(boundaryIndex, command.commandName()));
    }
    return List.copyOf(insertions);
  }

  private static int nearestBoundaryByProgress(
      List<PathSample> boundaries, double progress, PathPlannerPath path) {
    double bestDelta = Double.POSITIVE_INFINITY;
    int bestIndex = boundaries.size() - 1;
    for (int i = 0; i < boundaries.size(); i++) {
      double candidate = boundaries.get(i).progress();
      if (!Double.isFinite(candidate)) {
        candidate = estimateProgress(path, boundaries.get(i).distanceMeters());
      }
      double delta = Math.abs(candidate - progress);
      if (delta < bestDelta) {
        bestDelta = delta;
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private static int nearestBoundaryByTime(List<PathSample> boundaries, double timestampSeconds) {
    double bestDelta = Double.POSITIVE_INFINITY;
    int bestIndex = boundaries.size() - 1;
    for (int i = 0; i < boundaries.size(); i++) {
      double candidate = boundaries.get(i).timeSeconds();
      if (!Double.isFinite(candidate)) {
        continue;
      }
      double delta = Math.abs(candidate - timestampSeconds);
      if (delta < bestDelta) {
        bestDelta = delta;
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private static double estimateProgress(PathPlannerPath path, double distanceMeters) {
    double totalDistance =
        path.getAllPathPoints().isEmpty()
            ? 0.0
            : path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).distanceAlongPath;
    if (totalDistance <= 1e-6) {
      return path.getWaypoints().size() - 1.0;
    }
    return MathUtil.interpolate(
        0.0, path.getWaypoints().size() - 1.0, distanceMeters / totalDistance);
  }

  private static PathSegmentStep buildPathSegmentStep(
      String pathName,
      int segmentOrdinal,
      int segmentTotal,
      List<PathSample> boundaries,
      int startBoundaryIndex,
      int endBoundaryIndex,
      double maxVelocityMps,
      double endVelocityMps,
      boolean stopOnEnd) {
    if (startBoundaryIndex > endBoundaryIndex
        || startBoundaryIndex < 0
        || endBoundaryIndex < 0
        || endBoundaryIndex >= boundaries.size()) {
      return null;
    }
    ArrayList<PoseSpec> routeWaypoints = new ArrayList<>();
    for (int i = startBoundaryIndex; i < endBoundaryIndex; i++) {
      routeWaypoints.add(poseSpec(boundaries.get(i).pose()));
    }
    PathSample finalBoundary = boundaries.get(endBoundaryIndex);
    double segmentStartDistanceMeters =
        startBoundaryIndex == 0 ? 0.0 : boundaries.get(startBoundaryIndex - 1).distanceMeters();
    double routeLengthMeters =
        Math.max(0.0, finalBoundary.distanceMeters() - segmentStartDistanceMeters);
    String label =
        segmentTotal <= 1 ? pathName : pathName + " • " + segmentOrdinal + "/" + segmentTotal;
    StepSpec step =
        new StepSpec(
            "PATH",
            null,
            null,
            null,
            label,
            null,
            null,
            finalBoundary.pose().getX(),
            finalBoundary.pose().getY(),
            finalBoundary.pose().getRotation().getDegrees(),
            null,
            finiteOrNull(maxVelocityMps),
            routeLengthMeters,
            null,
            null,
            null,
            finiteOrNull(endVelocityMps),
            stopOnEnd,
            List.copyOf(routeWaypoints));
    return new PathSegmentStep(step);
  }

  private static StepSpec commandStep(String commandName, String label, PoseSpec boundaryPose) {
    return new StepSpec(
        "NAMED_COMMAND",
        null,
        null,
        commandName,
        label,
        null,
        null,
        boundaryPose == null ? null : boundaryPose.xMeters(),
        boundaryPose == null ? null : boundaryPose.yMeters(),
        boundaryPose == null ? null : boundaryPose.headingDeg(),
        null,
        null,
        0.0,
        null,
        null,
        null,
        null,
        Boolean.TRUE,
        List.of());
  }

  private static StepSpec waitStep(double waitSeconds, PoseSpec boundaryPose) {
    return new StepSpec(
        "WAIT",
        null,
        null,
        null,
        String.format(Locale.ROOT, "Wait %.2fs", waitSeconds),
        null,
        null,
        boundaryPose == null ? null : boundaryPose.xMeters(),
        boundaryPose == null ? null : boundaryPose.yMeters(),
        boundaryPose == null ? null : boundaryPose.headingDeg(),
        null,
        null,
        0.0,
        null,
        null,
        waitSeconds,
        null,
        Boolean.TRUE,
        List.of());
  }

  private ResolvedCommand resolveBoundaryCommand(JSONObject commandJson, String context)
      throws LoadFailure {
    String type = stringValue(commandJson.get("type"));
    JSONObject data = requireObject(commandJson.get("data"), context + " data");
    return switch (type) {
      case "named" -> new ResolvedCommand(requireNamedCommandName(data, context), null);
      case "wait" -> new ResolvedCommand(null, requireWaitSeconds(data, context));
      default ->
          throw new LoadFailure(
              context
                  + " uses unsupported command type \""
                  + type
                  + "\". Queue execution only supports named or wait boundary commands.");
    };
  }

  private String requireNamedCommandName(JSONObject data, String context) throws LoadFailure {
    String commandName = requireString(data.get("name"), context + " name");
    if (!NamedCommands.hasCommand(commandName)) {
      throw new LoadFailure(context + " references unknown named command \"" + commandName + "\".");
    }
    return commandName;
  }

  private static double requireWaitSeconds(JSONObject data, String context) throws LoadFailure {
    Double waitSeconds = nullableWaitSeconds(data.get("waitTime"));
    if (waitSeconds == null) {
      throw new LoadFailure(context + " is missing a numeric waitTime.");
    }
    return waitSeconds;
  }

  private static Double nullableWaitSeconds(Object value) {
    if (value instanceof Number number) {
      return number.doubleValue();
    }
    if (value instanceof JSONObject json) {
      Object nested = json.get("val");
      if (nested instanceof Number number) {
        return number.doubleValue();
      }
    }
    return null;
  }

  private static Rotation2d headingForPoint(
      List<PathPoint> points, int index, Rotation2d goalRotation) {
    PathPoint point = points.get(index);
    if (point.rotationTarget != null) {
      return point.rotationTarget.rotation();
    }
    if (index == points.size() - 1) {
      return goalRotation;
    }
    Rotation2d tangent = points.get(index + 1).position.minus(point.position).getAngle();
    if (Double.isFinite(tangent.getRadians())) {
      return tangent;
    }
    if (index > 0) {
      tangent = point.position.minus(points.get(index - 1).position).getAngle();
      if (Double.isFinite(tangent.getRadians())) {
        return tangent;
      }
    }
    return goalRotation;
  }

  private static PoseSpec poseSpec(Pose2d pose) {
    return new PoseSpec(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private static boolean isStopped(double endVelocityMps) {
    return !Double.isFinite(endVelocityMps) || Math.abs(endVelocityMps) <= STOPPED_END_VELOCITY_MPS;
  }

  private static double boundaryEndVelocityMps(PathSample boundary, double fallbackVelocityMps) {
    if (boundary == null) {
      return fallbackVelocityMps;
    }
    double boundaryVelocity = boundary.maxVelocityMps();
    if (Double.isFinite(boundaryVelocity) && boundaryVelocity > STOPPED_END_VELOCITY_MPS) {
      return boundaryVelocity;
    }
    return fallbackVelocityMps;
  }

  private static Double finiteOrNull(double value) {
    return Double.isFinite(value) ? value : null;
  }

  private static Optional<RawAutoDto> readRawAuto(Path autoFile) {
    try {
      return Optional.of(JSON.readValue(autoFile.toFile(), RawAutoDto.class));
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Failed to parse PathPlanner auto metadata from \"" + autoFile + "\".", false);
      return Optional.empty();
    }
  }

  private static JSONObject readPathJson(String pathName)
      throws IOException, ParseException, LoadFailure {
    return readJsonObject(
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve("pathplanner/paths/" + pathName + ".path"));
  }

  private static JSONObject readChoreoJson(String pathName)
      throws IOException, ParseException, LoadFailure {
    String choreoName = pathName;
    int splitSeparator = choreoName.lastIndexOf('.');
    if (splitSeparator > 0) {
      String suffix = choreoName.substring(splitSeparator + 1);
      try {
        Integer.parseInt(suffix);
        choreoName = choreoName.substring(0, splitSeparator);
      } catch (NumberFormatException ignored) {
        // Keep original name if the suffix is not a split index.
      }
    }
    return readJsonObject(
        Filesystem.getDeployDirectory().toPath().resolve("choreo/" + choreoName + ".traj"));
  }

  private static JSONObject readJsonObject(Path file)
      throws IOException, ParseException, LoadFailure {
    if (!Files.isRegularFile(file)) {
      throw new LoadFailure("Required deploy file was not found: " + file);
    }
    try (BufferedReader reader = new BufferedReader(new FileReader(file.toFile()))) {
      Object parsed = new JSONParser().parse(reader);
      return requireObject(parsed, file.toString());
    }
  }

  private static JSONObject requireObject(Object value, String context) throws LoadFailure {
    if (value instanceof JSONObject jsonObject) {
      return jsonObject;
    }
    throw new LoadFailure("Expected object for " + context + ".");
  }

  private static JSONArray requireArray(Object value, String context) throws LoadFailure {
    if (value instanceof JSONArray jsonArray) {
      return jsonArray;
    }
    throw new LoadFailure("Expected array for " + context + ".");
  }

  private static JSONObject objectValue(Object value) {
    return value instanceof JSONObject jsonObject ? jsonObject : null;
  }

  private static String requireString(Object value, String context) throws LoadFailure {
    String string = stringValue(value);
    if (string == null) {
      throw new LoadFailure("Expected string for " + context + ".");
    }
    return string;
  }

  private static String stringValue(Object value) {
    if (value instanceof String string) {
      return blankToNull(string);
    }
    return null;
  }

  private static double numericValue(Object value) throws LoadFailure {
    if (value instanceof Number number) {
      return number.doubleValue();
    }
    throw new LoadFailure("Expected numeric value.");
  }

  private static double nestedNumericValue(JSONObject object, String key, String nestedKey)
      throws LoadFailure {
    JSONObject nested = requireObject(object.get(key), key);
    return numericValue(nested.get(nestedKey));
  }

  private static String autoName(Path autoFile) {
    String fileName = autoFile.getFileName().toString();
    if (!fileName.toLowerCase(Locale.ROOT).endsWith(".auto")) {
      return null;
    }
    return fileName.substring(0, fileName.length() - 5);
  }

  private static String blankToNull(String value) {
    return value == null || value.isBlank() ? null : value.trim();
  }

  record LoadedAuto(
      String id,
      String name,
      String folder,
      String relativePath,
      Long updatedAt,
      PoseSpec startPose,
      List<ZoneSpec> customZones,
      List<StepSpec> steps) {}

  record PoseSpec(double xMeters, double yMeters, double headingDeg) {}

  record ZoneSpec(
      String id,
      String label,
      double xMinMeters,
      double yMinMeters,
      double xMaxMeters,
      double yMaxMeters,
      boolean locked) {}

  record StepSpec(
      String type,
      String spotId,
      String requestedState,
      String commandName,
      String label,
      String group,
      String alliance,
      Double xMeters,
      Double yMeters,
      Double headingDeg,
      Double constraintFactor,
      Double maxVelocityMps,
      Double routeLengthMeters,
      Double toleranceMeters,
      Double timeoutSeconds,
      Double waitSeconds,
      Double endVelocityMps,
      Boolean stopOnEnd,
      List<PoseSpec> routeWaypoints) {}

  private record ConvertedAuto(PoseSpec startPose, List<StepSpec> steps) {}

  private record ConvertedPath(PoseSpec startPose, PoseSpec endPose, List<StepSpec> steps) {}

  private record PathSegmentStep(StepSpec step) {}

  private record PathSample(
      Pose2d pose,
      double distanceMeters,
      double progress,
      double maxVelocityMps,
      double timeSeconds) {}

  private record MarkerInsertion(int boundaryIndex, String commandName) {}

  private record ResolvedCommand(String commandName, Double waitSeconds) {
    boolean isNamedCommand() {
      return commandName != null && !commandName.isBlank();
    }
  }

  private static final class ConversionContext {
    private PoseSpec startPose;
    private PoseSpec currentBoundaryPose;
  }

  private static final class LoadFailure extends Exception {
    LoadFailure(String message) {
      super(message);
    }
  }

  private record LibraryIndexDto(
      String version, String generator, List<AutoManifestEntryDto> autos) {}

  private record AutoManifestEntryDto(
      String id, String name, String folder, String relativePath, Long updatedAt) {}

  private static final class RawAutoDto {
    public String folder;
  }
}
