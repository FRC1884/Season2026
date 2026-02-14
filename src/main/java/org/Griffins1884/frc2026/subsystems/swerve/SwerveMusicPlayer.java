package org.Griffins1884.frc2026.subsystems.swerve;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveMusicPlayer {
  private final Orchestra orchestra = new Orchestra();
  private final Path musicPath;
  private final boolean hasInstruments;

  public SwerveMusicPlayer(Module[] modules, String relativeMusicPath) {
    List<TalonFX> instruments = new ArrayList<>();
    if (modules != null) {
      for (Module module : modules) {
        if (module != null) {
          module.addOrchestraInstruments(instruments);
        }
      }
    }
    for (TalonFX motor : instruments) {
      orchestra.addInstrument(motor);
    }
    hasInstruments = !instruments.isEmpty();
    musicPath = Filesystem.getDeployDirectory().toPath().resolve(relativeMusicPath);
  }

  public void start() {
    if (!hasInstruments) {
      Logger.recordOutput("Swerve/Music/Status", "NoInstruments");
      return;
    }
    if (!musicPath.toFile().exists()) {
      Logger.recordOutput("Swerve/Music/Status", "FileMissing");
      Logger.recordOutput("Swerve/Music/Path", musicPath.toString());
      return;
    }
    StatusCode status = orchestra.loadMusic(musicPath.toString());
    Logger.recordOutput("Swerve/Music/LoadStatus", status.toString());
    if (status.isOK()) {
      orchestra.play();
    }
  }

  public void stop() {
    orchestra.stop();
  }
}
