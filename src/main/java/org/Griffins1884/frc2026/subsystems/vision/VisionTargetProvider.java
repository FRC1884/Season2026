package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

public interface VisionTargetProvider {
  Optional<Translation2d> getBestTargetTranslation(Pose2d robotPose);
}
