package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

public interface VisionTargetProvider {
  Optional<Translation2d> getBestTargetTranslation(Pose2d robotPose);
}
