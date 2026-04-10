package org.Griffins1884.frc2026.subsystems.objectivetracker;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.Griffins1884.frc2026.commands.AutoAlignToPoseCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class RebuiltAutoQueueTest {
  private static final ObjectMapper JSON = new ObjectMapper();
  private final List<Path> createdFiles = new ArrayList<>();

  @AfterEach
  void cleanup() throws IOException {
    NamedCommands.clearAll();
    com.pathplanner.lib.path.PathPlannerPath.clearCache();
    for (Path file : createdFiles) {
      DeployAutoTestSupport.deleteIfExists(file);
    }
  }

  @Test
  void selectedDeployAutoBuildsQueueNativeCommandAndKeepsPreviewOrder() throws Exception {
    NamedCommands.registerCommand("Intaking", Commands.none());
    String pathName = DeployAutoTestSupport.uniqueName("queue-path");
    String autoName = DeployAutoTestSupport.uniqueName("queue-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(pathName, 0.0, "Intaking"));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(pathName))));

    RebuiltAutoQueue queue = DeployAutoTestSupport.newQueue(DeployAutoTestSupport.fakeDrive());
    selectAuto(queue, autoName);

    Command autonomous = queue.createAutonomousCommand();
    assertNotNull(autonomous);

    List<Command> allCommands = DeployAutoTestSupport.collectCommands(autonomous);
    assertTrue(
        allCommands.stream()
            .noneMatch(command -> command.getClass().getName().contains("PathPlannerAuto")));
    assertTrue(allCommands.stream().anyMatch(AutoAlignToPoseCommand.class::isInstance));

    JsonNode queueState = JSON.readTree(queue.getQueueStateJson());
    List<String> labels = new ArrayList<>();
    queueState.get("steps").forEach(step -> labels.add(step.get("label").asText()));
    assertEquals(
        List.of(pathName + " • 1/2", "Intaking", pathName + " • 2/2"), labels.subList(0, 3));
  }

  @Test
  void finalSegmentStopIntentTracksPathEndVelocity() throws Exception {
    String stopPath = DeployAutoTestSupport.uniqueName("stop-path");
    String stopAuto = DeployAutoTestSupport.uniqueName("stop-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(stopPath, 0.0, null));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            stopAuto,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(stopPath))));

    String flowPath = DeployAutoTestSupport.uniqueName("flow-path");
    String flowAuto = DeployAutoTestSupport.uniqueName("flow-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(flowPath, 1.2, null));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            flowAuto,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(flowPath))));

    assertTrue(lastAutoAlignStopOnEnd(stopAuto));
    assertFalse(lastAutoAlignStopOnEnd(flowAuto));
  }

  @Test
  void deployAutoPhaseTransitionsStillWorkForBoundaryOnlyAutos() throws Exception {
    NamedCommands.registerCommand("Shoot", Commands.none());
    String autoName = DeployAutoTestSupport.uniqueName("wait-auto");
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                false,
                DeployAutoTestSupport.waitCommandJson(0.1),
                DeployAutoTestSupport.namedCommandJson("Shoot"))));

    RebuiltAutoQueue queue = DeployAutoTestSupport.newQueue(DeployAutoTestSupport.fakeDrive());
    selectAuto(queue, autoName);

    Command autonomous = queue.createAutonomousCommand();
    assertNotNull(autonomous);

    autonomous.initialize();
    assertEquals("RUNNING", queue.getPhaseName());

    autonomous.end(false);
    assertEquals("COMPLETE", queue.getPhaseName());
  }

  private static void selectAuto(RebuiltAutoQueue queue, String autoName) {
    OperatorBoardIO.OperatorBoardIOInputs inputs = new OperatorBoardIO.OperatorBoardIOInputs();
    inputs.selectedAutoId = new String[] {autoName};
    queue.handleInputs(inputs);
  }

  private static boolean lastAutoAlignStopOnEnd(String autoName) throws Exception {
    RebuiltAutoQueue queue = DeployAutoTestSupport.newQueue(DeployAutoTestSupport.fakeDrive());
    selectAuto(queue, autoName);
    Command autonomous = queue.createAutonomousCommand();
    assertNotNull(autonomous);

    List<AutoAlignToPoseCommand> alignCommands =
        DeployAutoTestSupport.collectCommands(autonomous).stream()
            .filter(AutoAlignToPoseCommand.class::isInstance)
            .map(AutoAlignToPoseCommand.class::cast)
            .collect(Collectors.toList());
    assertFalse(alignCommands.isEmpty());
    AutoAlignToPoseCommand last = alignCommands.get(alignCommands.size() - 1);
    Field stopOnEndField = AutoAlignToPoseCommand.class.getDeclaredField("stopOnEnd");
    stopOnEndField.setAccessible(true);
    return stopOnEndField.getBoolean(last);
  }
}
