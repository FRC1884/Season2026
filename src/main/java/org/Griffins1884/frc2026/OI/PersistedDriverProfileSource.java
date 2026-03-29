package org.Griffins1884.frc2026.OI;

import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardDataModels;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardPersistence;

final class PersistedDriverProfileSource {
  private static final double REFRESH_PERIOD_SEC = 0.35;

  private final OperatorBoardPersistence persistence;
  private final String fallbackControllerType;
  private final int fallbackPort;

  private double lastRefreshTimestampSec = Double.NEGATIVE_INFINITY;
  private ResolvedProfile cachedProfile;

  PersistedDriverProfileSource(String fallbackControllerType, int fallbackPort) {
    this.persistence = new OperatorBoardPersistence();
    this.persistence.initialize();
    this.fallbackControllerType = fallbackControllerType;
    this.fallbackPort = Math.max(0, fallbackPort);
    this.cachedProfile = resolveProfile(persistence.readJoystickMappings());
  }

  synchronized ResolvedProfile getActiveProfile() {
    double now = Timer.getFPGATimestamp();
    if (cachedProfile != null && now - lastRefreshTimestampSec < REFRESH_PERIOD_SEC) {
      return cachedProfile;
    }
    cachedProfile = resolveProfile(persistence.readJoystickMappings());
    lastRefreshTimestampSec = now;
    return cachedProfile;
  }

  private ResolvedProfile resolveProfile(
      OperatorBoardDataModels.JoystickMappingsDocument document) {
    List<OperatorBoardDataModels.JoystickProfile> profiles = document.profiles();
    OperatorBoardDataModels.JoystickProfile activeProfile =
        profiles == null || profiles.isEmpty()
            ? null
            : profiles.stream()
                .filter(profile -> profile.id().equals(document.activeProfileId()))
                .findFirst()
                .or(
                    () ->
                        profiles.stream()
                            .filter(OperatorBoardDataModels.JoystickProfile::enabled)
                            .findFirst())
                .orElse(profiles.get(0));

    DriverControllerLayout layout =
        DriverControllerLayout.fromProfileType(
            activeProfile != null ? activeProfile.controllerType() : fallbackControllerType);
    int controllerPort =
        activeProfile == null ? fallbackPort : Math.max(activeProfile.controllerPort(), 0);
    Map<String, OperatorBoardDataModels.JoystickBinding> bindingsByTarget = new LinkedHashMap<>();
    if (activeProfile != null && activeProfile.bindings() != null) {
      for (OperatorBoardDataModels.JoystickBinding binding : activeProfile.bindings()) {
        if (binding == null || binding.targetId() == null || binding.targetId().isBlank()) {
          continue;
        }
        String inputKind = DriverControllerLayout.normalizeInputKind(binding.inputKind());
        String canonicalInputId = layout.canonicalInputId(inputKind, binding.inputId());
        bindingsByTarget.put(
            binding.targetId(),
            new OperatorBoardDataModels.JoystickBinding(
                inputKind,
                canonicalInputId,
                binding.displayLabel(),
                binding.targetType(),
                binding.targetId(),
                binding.targetLabel(),
                binding.enabled(),
                binding.notes()));
      }
    }

    return new ResolvedProfile(
        activeProfile != null ? activeProfile.id() : "fallback-driver",
        activeProfile != null ? activeProfile.name() : "Fallback Driver",
        layout,
        controllerPort,
        bindingsByTarget);
  }

  record ResolvedProfile(
      String id,
      String name,
      DriverControllerLayout controllerLayout,
      int controllerPort,
      Map<String, OperatorBoardDataModels.JoystickBinding> bindingsByTarget) {
    OperatorBoardDataModels.JoystickBinding bindingForTarget(String targetId) {
      return bindingsByTarget.get(targetId);
    }
  }
}
