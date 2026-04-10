package org.Griffins1884.frc2026.subsystems.objectivetracker;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class DeployAutoLibraryTest {
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
  void loadsNormalPathPlannerAutoIntoExecutionSteps() throws Exception {
    NamedCommands.registerCommand("Intaking", Commands.none());
    String pathName = DeployAutoTestSupport.uniqueName("library-path");
    String autoName = DeployAutoTestSupport.uniqueName("library-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(pathName, 0.0, "Intaking"));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(pathName))));

    DeployAutoLibrary.LoadedAuto loadedAuto =
        DeployAutoTestSupport.newLibrary().loadAuto(autoName).orElseThrow();

    assertTrue(loadedAuto.steps().size() >= 3);
    assertEquals("PATH", loadedAuto.steps().get(0).type());
    assertEquals("NAMED_COMMAND", loadedAuto.steps().get(1).type());
    assertEquals("Intaking", loadedAuto.steps().get(1).commandName());
    assertEquals("PATH", loadedAuto.steps().get(2).type());
    assertTrue(loadedAuto.startPose() != null);
  }

  @Test
  void preservesChoreoAutoSupport() throws Exception {
    NamedCommands.registerCommand("Intaking", Commands.none());
    NamedCommands.registerCommand("Intake", Commands.none());
    String autoName = DeployAutoTestSupport.uniqueName("choreo-auto");
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                true, DeployAutoTestSupport.pathCommandJson("RightOverBump"))));

    DeployAutoLibrary.LoadedAuto loadedAuto =
        DeployAutoTestSupport.newLibrary().loadAuto(autoName).orElseThrow();

    assertFalse(loadedAuto.steps().isEmpty());
    assertTrue(
        loadedAuto.steps().stream()
            .anyMatch(
                step -> "PATH".equals(step.type()) && step.label().startsWith("RightOverBump")));
  }

  @Test
  void extractsEndVelocityAndStopIntent() throws Exception {
    String pathName = DeployAutoTestSupport.uniqueName("velocity-path");
    String autoName = DeployAutoTestSupport.uniqueName("velocity-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(pathName, 1.35, null));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(pathName))));

    DeployAutoLibrary.LoadedAuto loadedAuto =
        DeployAutoTestSupport.newLibrary().loadAuto(autoName).orElseThrow();
    DeployAutoLibrary.StepSpec finalPathStep =
        loadedAuto.steps().stream()
            .filter(step -> "PATH".equals(step.type()))
            .reduce((first, second) -> second)
            .orElseThrow();

    assertEquals(1.35, finalPathStep.endVelocityMps(), 1e-6);
    assertEquals(Boolean.FALSE, finalPathStep.stopOnEnd());
  }

  @Test
  void failsCleanlyOnUnknownMarkerNames() throws Exception {
    String pathName = DeployAutoTestSupport.uniqueName("unknown-marker-path");
    String autoName = DeployAutoTestSupport.uniqueName("unknown-marker-auto");
    createdFiles.add(DeployAutoTestSupport.writePath(pathName, 0.0, "DoesNotExist"));
    createdFiles.add(
        DeployAutoTestSupport.writeAuto(
            autoName,
            DeployAutoTestSupport.sequentialAutoJson(
                false, DeployAutoTestSupport.pathCommandJson(pathName))));

    DeployAutoLibrary library = DeployAutoTestSupport.newLibrary();

    assertTrue(library.loadAuto(autoName).isEmpty());
    assertTrue(library.getLastFailureMessage().contains("DoesNotExist"));
  }
}
