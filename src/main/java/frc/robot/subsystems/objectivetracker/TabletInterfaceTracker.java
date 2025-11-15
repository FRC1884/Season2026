package frc.robot.subsystems.objectivetracker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TabletInterfaceTracker extends SubsystemBase {

  private final ReefControlsIO io;
  private final ReefControlsIOInputsAutoLogged inputs = new ReefControlsIOInputsAutoLogged();

  private boolean buttonClicked = false;
  private int selectedLevel = 0;
  private int level1Coral = 0;
  private int level2Coral = 0;
  private int level3Coral = 0;
  private int level4Coral = 0;
  private int algae = 0;

  public TabletInterfaceTracker(ReefControlsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("ReefControls", inputs);

    /*if (inputs.algaeState.length > 0) {
      buttonClicked = inputs.algaeState[0] > 0;
    }*/
    updateTablet();
  }

  private void updateTablet() {
    if (inputs.selectedLevel.length > 0) {
      selectedLevel = inputs.selectedLevel[0];
    }
    if (inputs.level1State.length > 0) {
      level1Coral = inputs.level1State[0];
    }
    if (inputs.level2State.length > 0) {
      level2Coral = inputs.level2State[0];
    }
    if (inputs.level3State.length > 0) {
      level3Coral = inputs.level3State[0];
    }
    if (inputs.level4State.length > 0) {
      level4Coral = inputs.level4State[0];
    }

    io.setSelectedLevel(selectedLevel);
    io.setLevel1State(level1Coral);
    io.setLevel2State(level2Coral);
    io.setLevel3State(level3Coral);
    io.setLevel4State(level4Coral);

    if (inputs.algaeState.length > 0) {
      algae = inputs.algaeState[0];
    }

    io.setAlgaeState(algae);
  }

  // public boolean buttonClicked() {return buttonClicked}

}
