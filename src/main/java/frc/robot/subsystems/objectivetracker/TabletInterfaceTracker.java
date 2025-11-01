package frc.robot.subsystems.objectivetracker;

public class TabletInterfaceTracker {

  private final ReefControlsIO io;
  // private final ReefControlsIOInputsAutoLogged inputs = new ReefControlsIOInputsAutoLogged();

  private boolean buttonClicked = false;

  public TabletInterfaceTracker(ReefControlsIO io) {
    this.io = io;
  }

  public void Periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("ReefControls", inputs);

    // if (inputs.algaeState.length > 0) {
    //     buttonClicked = inputs.algaeState[0] > 0;
    // }
  }

  public boolean buttonClicked() {
    return buttonClicked;
  }
}
