package org.Griffins1884.frc2026.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface OperatorBoardIO {
  @AutoLog
  class OperatorBoardIOInputs {
    public String[] requestedState = new String[] {}; // Superstructure.SuperState name
    public boolean autoStateEnableRequested = false;
    public boolean playSwerveMusicRequested = false;
    public boolean stopSwerveMusicRequested = false;
    public double swerveMusicVolume = Double.NaN;
    public boolean rollLogsRequested = false;
    public boolean cleanLogsRequested = false;
    public boolean requestIntakeDeployRezero = false;
    public boolean cancelIntakeDeployRezero = false;
    public boolean requestManualIntakeDeployZeroSeek = false;
    public boolean cancelManualIntakeDeployZeroSeek = false;
    public String[] selectedAutoId = new String[] {};
    public String[] autoQueueSpec = new String[] {};
    public String[] autoQueueCommand = new String[] {};
    public String[] runtimeProfileSpec = new String[] {};
    public boolean applyRuntimeProfile = false;
    public boolean resetRuntimeProfile = false;
  }

  default void updateInputs(OperatorBoardIOInputs inputs) {}

  default void setRequestedState(String value) {}

  default void setCurrentState(String value) {}

  default void setRequestAccepted(boolean value) {}

  default void setRequestReason(String value) {}

  default void setTargetType(String value) {}

  default void setTargetPose(double[] value) {}

  default void setTargetPoseValid(boolean value) {}

  default void setRobotPose(double[] value) {}

  default void setAutoQueueState(String value) {}

  default void setAutoQueuePreviewPose(double[] value) {}

  default void setAutoQueuePreviewPoseValid(boolean value) {}

  default void setSelectedAutoState(String value) {}

  default void setRuntimeProfileState(String value) {}

  default void setRuntimeProfileStatus(String value) {}

  default void setSystemCheckState(String value) {}

  default void setAutoCheckState(String value) {}

  default void setAutoQuickRunState(String value) {}

  default void setNtDiagnosticsState(String value) {}

  default void setMechanismStatusState(String value) {}

  default void setActionTraceState(String value) {}

  default void setHasBall(boolean value) {}

  default void setDsMode(String value) {}

  default void setBatteryVoltage(double value) {}

  default void setBrownout(boolean value) {}

  default void setAlliance(String value) {}

  default void setMatchTime(double value) {}

  default void setHubTimeframe(String value) {}

  default void setHubStatusValid(boolean value) {}

  default void setRedHubStatus(String value) {}

  default void setBlueHubStatus(String value) {}

  default void setOurHubStatus(String value) {}

  default void setOurHubActive(boolean value) {}

  default void setAutoWinnerAlliance(String value) {}

  default void setGameDataRaw(String value) {}

  default void setHubRecommendation(String value) {}

  default void setTurretAtSetpoint(boolean value) {}

  default void setTurretMode(String value) {}

  default void setSysIdDrivePhase(String value) {}

  default void setSysIdDriveActive(boolean value) {}

  default void setSysIdDriveLastCompleted(double value) {}

  default void setSysIdDriveLastCompletedPhase(String value) {}

  default void setSysIdTurnPhase(String value) {}

  default void setSysIdTurnActive(boolean value) {}

  default void setSysIdTurnLastCompleted(double value) {}

  default void setSysIdTurnLastCompletedPhase(String value) {}

  default void setVisionStatus(String value) {}

  default void setVisionPoseVisible(boolean value) {}

  default void setShootEnabled(boolean value) {}

  default void setIntakeRollersHeld(boolean value) {}

  default void setIntakeDeployed(boolean value) {}

  default void setTeleopOverrideActive(boolean value) {}

  default void setDriverControllerControlActive(boolean value) {}

  default void setShootReadyLatched(boolean value) {}

  default void setIntakeDeployRezeroInProgress(boolean value) {}

  default void setManualIntakeDeployZeroSeekInProgress(boolean value) {}

  default void setLogRollStatus(String value) {}

  default void setLogRollLastTimestamp(double value) {}

  default void setLogRollCount(int value) {}

  default void setLogCleanStatus(String value) {}

  default void setLogCleanLastTimestamp(double value) {}

  default void setLogCleanCount(int value) {}

  default void setLogCleanDeletedEntries(int value) {}
}
