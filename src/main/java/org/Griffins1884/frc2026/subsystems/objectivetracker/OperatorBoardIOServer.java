// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.Griffins1884.frc2026.subsystems.objectivetracker;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedBoolean;

public class OperatorBoardIOServer implements OperatorBoardIO {
  private final StringSubscriber requestedStateIn;
  private final BooleanSubscriber autoStateEnableIn;
  private final BooleanSubscriber playSwerveMusicIn;
  private final BooleanSubscriber stopSwerveMusicIn;
  private final DoubleSubscriber swerveMusicVolumeIn;
  private final BooleanSubscriber rollLogsIn;
  private final BooleanSubscriber cleanLogsIn;

  private final StringPublisher requestedStateOut;
  private final StringPublisher currentStateOut;
  private final BooleanPublisher requestAcceptedOut;
  private final StringPublisher requestReasonOut;
  private final StringPublisher targetTypeOut;
  private final DoubleArrayPublisher targetPoseOut;
  private final BooleanPublisher targetPoseValidOut;
  private final DoubleArrayPublisher robotPoseOut;
  private final BooleanPublisher hasBallOut;
  private final StringPublisher dsModeOut;
  private final DoublePublisher batteryVoltageOut;
  private final BooleanPublisher brownoutOut;
  private final StringPublisher allianceOut;
  private final DoublePublisher matchTimeOut;
  private final StringPublisher hubTimeframeOut;
  private final BooleanPublisher hubStatusValidOut;
  private final StringPublisher redHubStatusOut;
  private final StringPublisher blueHubStatusOut;
  private final StringPublisher ourHubStatusOut;
  private final BooleanPublisher ourHubActiveOut;
  private final StringPublisher autoWinnerAllianceOut;
  private final StringPublisher gameDataRawOut;
  private final StringPublisher hubRecommendationOut;
  private final BooleanPublisher turretAtSetpointOut;
  private final StringPublisher turretModeOut;
  private final StringPublisher sysIdDrivePhaseOut;
  private final BooleanPublisher sysIdDriveActiveOut;
  private final DoublePublisher sysIdDriveLastCompletedOut;
  private final StringPublisher sysIdDriveLastCompletedPhaseOut;
  private final StringPublisher sysIdTurnPhaseOut;
  private final BooleanPublisher sysIdTurnActiveOut;
  private final DoublePublisher sysIdTurnLastCompletedOut;
  private final StringPublisher sysIdTurnLastCompletedPhaseOut;
  private final StringPublisher visionStatusOut;
  private final StringPublisher logRollStatusOut;
  private final DoublePublisher logRollLastTimestampOut;
  private final IntegerPublisher logRollCountOut;
  private final StringPublisher logCleanStatusOut;
  private final DoublePublisher logCleanLastTimestampOut;
  private final IntegerPublisher logCleanCountOut;
  private final IntegerPublisher logCleanDeletedEntriesOut;

  public OperatorBoardIOServer() {
    var inputTable = NetworkTableInstance.getDefault().getTable(OperatorBoardContract.TO_ROBOT);
    requestedStateIn =
        inputTable
            .getStringTopic(OperatorBoardContract.REQUESTED_STATE)
            .subscribe("", PubSubOption.keepDuplicates(true));
    autoStateEnableIn =
        inputTable
            .getBooleanTopic(OperatorBoardContract.AUTO_STATE_ENABLE)
            .subscribe(false, PubSubOption.keepDuplicates(true));
    playSwerveMusicIn =
        inputTable
            .getBooleanTopic(OperatorBoardContract.PLAY_SWERVE_MUSIC)
            .subscribe(false, PubSubOption.keepDuplicates(true));
    stopSwerveMusicIn =
        inputTable
            .getBooleanTopic(OperatorBoardContract.STOP_SWERVE_MUSIC)
            .subscribe(false, PubSubOption.keepDuplicates(true));
    swerveMusicVolumeIn =
        inputTable
            .getDoubleTopic(OperatorBoardContract.SWERVE_MUSIC_VOLUME)
            .subscribe(Double.NaN, PubSubOption.keepDuplicates(true));
    rollLogsIn =
        inputTable
            .getBooleanTopic(OperatorBoardContract.ROLL_LOGS)
            .subscribe(false, PubSubOption.keepDuplicates(true));
    cleanLogsIn =
        inputTable
            .getBooleanTopic(OperatorBoardContract.CLEAN_LOGS)
            .subscribe(false, PubSubOption.keepDuplicates(true));

    var outputTable =
        NetworkTableInstance.getDefault().getTable(OperatorBoardContract.TO_DASHBOARD);
    requestedStateOut = outputTable.getStringTopic(OperatorBoardContract.REQUESTED_STATE).publish();
    currentStateOut = outputTable.getStringTopic(OperatorBoardContract.CURRENT_STATE).publish();
    requestAcceptedOut =
        outputTable.getBooleanTopic(OperatorBoardContract.REQUEST_ACCEPTED).publish();
    requestReasonOut = outputTable.getStringTopic(OperatorBoardContract.REQUEST_REASON).publish();
    targetTypeOut = outputTable.getStringTopic(OperatorBoardContract.TARGET_TYPE).publish();
    targetPoseOut = outputTable.getDoubleArrayTopic(OperatorBoardContract.TARGET_POSE).publish();
    targetPoseValidOut =
        outputTable.getBooleanTopic(OperatorBoardContract.TARGET_POSE_VALID).publish();
    robotPoseOut = outputTable.getDoubleArrayTopic(OperatorBoardContract.ROBOT_POSE).publish();
    hasBallOut = outputTable.getBooleanTopic(OperatorBoardContract.HAS_BALL).publish();
    dsModeOut = outputTable.getStringTopic(OperatorBoardContract.DS_MODE).publish();
    batteryVoltageOut = outputTable.getDoubleTopic(OperatorBoardContract.BATTERY_VOLTAGE).publish();
    brownoutOut = outputTable.getBooleanTopic(OperatorBoardContract.BROWNOUT).publish();
    allianceOut = outputTable.getStringTopic(OperatorBoardContract.ALLIANCE).publish();
    matchTimeOut = outputTable.getDoubleTopic(OperatorBoardContract.MATCH_TIME).publish();
    hubTimeframeOut = outputTable.getStringTopic(OperatorBoardContract.HUB_TIMEFRAME).publish();
    hubStatusValidOut =
        outputTable.getBooleanTopic(OperatorBoardContract.HUB_STATUS_VALID).publish();
    redHubStatusOut = outputTable.getStringTopic(OperatorBoardContract.RED_HUB_STATUS).publish();
    blueHubStatusOut = outputTable.getStringTopic(OperatorBoardContract.BLUE_HUB_STATUS).publish();
    ourHubStatusOut = outputTable.getStringTopic(OperatorBoardContract.OUR_HUB_STATUS).publish();
    ourHubActiveOut = outputTable.getBooleanTopic(OperatorBoardContract.OUR_HUB_ACTIVE).publish();
    autoWinnerAllianceOut =
        outputTable.getStringTopic(OperatorBoardContract.AUTO_WINNER_ALLIANCE).publish();
    gameDataRawOut = outputTable.getStringTopic(OperatorBoardContract.GAME_DATA_RAW).publish();
    hubRecommendationOut =
        outputTable.getStringTopic(OperatorBoardContract.HUB_RECOMMENDATION).publish();
    turretAtSetpointOut =
        outputTable.getBooleanTopic(OperatorBoardContract.TURRET_AT_SETPOINT).publish();
    turretModeOut = outputTable.getStringTopic(OperatorBoardContract.TURRET_MODE).publish();
    sysIdDrivePhaseOut =
        outputTable.getStringTopic(OperatorBoardContract.SYSID_DRIVE_PHASE).publish();
    sysIdDriveActiveOut =
        outputTable.getBooleanTopic(OperatorBoardContract.SYSID_DRIVE_ACTIVE).publish();
    sysIdDriveLastCompletedOut =
        outputTable.getDoubleTopic(OperatorBoardContract.SYSID_DRIVE_LAST_COMPLETED).publish();
    sysIdDriveLastCompletedPhaseOut =
        outputTable
            .getStringTopic(OperatorBoardContract.SYSID_DRIVE_LAST_COMPLETED_PHASE)
            .publish();
    sysIdTurnPhaseOut =
        outputTable.getStringTopic(OperatorBoardContract.SYSID_TURN_PHASE).publish();
    sysIdTurnActiveOut =
        outputTable.getBooleanTopic(OperatorBoardContract.SYSID_TURN_ACTIVE).publish();
    sysIdTurnLastCompletedOut =
        outputTable.getDoubleTopic(OperatorBoardContract.SYSID_TURN_LAST_COMPLETED).publish();
    sysIdTurnLastCompletedPhaseOut =
        outputTable.getStringTopic(OperatorBoardContract.SYSID_TURN_LAST_COMPLETED_PHASE).publish();
    visionStatusOut = outputTable.getStringTopic(OperatorBoardContract.VISION_STATUS).publish();
    logRollStatusOut = outputTable.getStringTopic(OperatorBoardContract.LOG_ROLL_STATUS).publish();
    logRollLastTimestampOut =
        outputTable.getDoubleTopic(OperatorBoardContract.LOG_ROLL_LAST_TIMESTAMP).publish();
    logRollCountOut = outputTable.getIntegerTopic(OperatorBoardContract.LOG_ROLL_COUNT).publish();
    logCleanStatusOut =
        outputTable.getStringTopic(OperatorBoardContract.LOG_CLEAN_STATUS).publish();
    logCleanLastTimestampOut =
        outputTable.getDoubleTopic(OperatorBoardContract.LOG_CLEAN_LAST_TIMESTAMP).publish();
    logCleanCountOut = outputTable.getIntegerTopic(OperatorBoardContract.LOG_CLEAN_COUNT).publish();
    logCleanDeletedEntriesOut =
        outputTable.getIntegerTopic(OperatorBoardContract.LOG_CLEAN_DELETED_ENTRIES).publish();
  }

  @Override
  public void updateInputs(OperatorBoardIOInputs inputs) {
    inputs.requestedState =
        requestedStateIn.readQueue().length > 0
            ? new String[] {requestedStateIn.get()}
            : new String[] {};
    TimestampedBoolean[] autoQueue = autoStateEnableIn.readQueue();
    if (autoQueue.length > 0) {
      inputs.autoStateEnableRequested = autoQueue[autoQueue.length - 1].value;
    } else {
      inputs.autoStateEnableRequested = false;
    }
    TimestampedBoolean[] musicQueue = playSwerveMusicIn.readQueue();
    if (musicQueue.length > 0) {
      inputs.playSwerveMusicRequested = musicQueue[musicQueue.length - 1].value;
    } else {
      inputs.playSwerveMusicRequested = false;
    }
    TimestampedBoolean[] stopQueue = stopSwerveMusicIn.readQueue();
    if (stopQueue.length > 0) {
      inputs.stopSwerveMusicRequested = stopQueue[stopQueue.length - 1].value;
    } else {
      inputs.stopSwerveMusicRequested = false;
    }
    var volumeQueue = swerveMusicVolumeIn.readQueue();
    if (volumeQueue.length > 0) {
      inputs.swerveMusicVolume = volumeQueue[volumeQueue.length - 1].value;
    } else {
      inputs.swerveMusicVolume = Double.NaN;
    }
    TimestampedBoolean[] rollQueue = rollLogsIn.readQueue();
    if (rollQueue.length > 0) {
      inputs.rollLogsRequested = rollQueue[rollQueue.length - 1].value;
    } else {
      inputs.rollLogsRequested = false;
    }
    TimestampedBoolean[] cleanQueue = cleanLogsIn.readQueue();
    if (cleanQueue.length > 0) {
      inputs.cleanLogsRequested = cleanQueue[cleanQueue.length - 1].value;
    } else {
      inputs.cleanLogsRequested = false;
    }
  }

  @Override
  public void setRequestedState(String value) {
    requestedStateOut.set(value == null ? "" : value);
  }

  @Override
  public void setCurrentState(String value) {
    currentStateOut.set(value == null ? "" : value);
  }

  @Override
  public void setRequestAccepted(boolean value) {
    requestAcceptedOut.set(value);
  }

  @Override
  public void setRequestReason(String value) {
    requestReasonOut.set(value == null ? "" : value);
  }

  @Override
  public void setTargetType(String value) {
    targetTypeOut.set(value == null ? "" : value);
  }

  @Override
  public void setTargetPose(double[] value) {
    targetPoseOut.set(value == null ? new double[] {} : value);
  }

  @Override
  public void setTargetPoseValid(boolean value) {
    targetPoseValidOut.set(value);
  }

  @Override
  public void setRobotPose(double[] value) {
    robotPoseOut.set(value == null ? new double[] {} : value);
  }

  @Override
  public void setHasBall(boolean value) {
    hasBallOut.set(value);
  }

  @Override
  public void setDsMode(String value) {
    dsModeOut.set(value == null ? "" : value);
  }

  @Override
  public void setBatteryVoltage(double value) {
    batteryVoltageOut.set(value);
  }

  @Override
  public void setBrownout(boolean value) {
    brownoutOut.set(value);
  }

  @Override
  public void setAlliance(String value) {
    allianceOut.set(value == null ? "" : value);
  }

  @Override
  public void setMatchTime(double value) {
    matchTimeOut.set(value);
  }

  @Override
  public void setHubTimeframe(String value) {
    hubTimeframeOut.set(value == null ? "" : value);
  }

  @Override
  public void setHubStatusValid(boolean value) {
    hubStatusValidOut.set(value);
  }

  @Override
  public void setRedHubStatus(String value) {
    redHubStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setBlueHubStatus(String value) {
    blueHubStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setOurHubStatus(String value) {
    ourHubStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setOurHubActive(boolean value) {
    ourHubActiveOut.set(value);
  }

  @Override
  public void setAutoWinnerAlliance(String value) {
    autoWinnerAllianceOut.set(value == null ? "" : value);
  }

  @Override
  public void setGameDataRaw(String value) {
    gameDataRawOut.set(value == null ? "" : value);
  }

  @Override
  public void setHubRecommendation(String value) {
    hubRecommendationOut.set(value == null ? "" : value);
  }

  @Override
  public void setTurretAtSetpoint(boolean value) {
    turretAtSetpointOut.set(value);
  }

  @Override
  public void setTurretMode(String value) {
    turretModeOut.set(value == null ? "" : value);
  }

  @Override
  public void setSysIdDrivePhase(String value) {
    sysIdDrivePhaseOut.set(value == null ? "" : value);
  }

  @Override
  public void setSysIdDriveActive(boolean value) {
    sysIdDriveActiveOut.set(value);
  }

  @Override
  public void setSysIdDriveLastCompleted(double value) {
    sysIdDriveLastCompletedOut.set(value);
  }

  @Override
  public void setSysIdDriveLastCompletedPhase(String value) {
    sysIdDriveLastCompletedPhaseOut.set(value == null ? "" : value);
  }

  @Override
  public void setSysIdTurnPhase(String value) {
    sysIdTurnPhaseOut.set(value == null ? "" : value);
  }

  @Override
  public void setSysIdTurnActive(boolean value) {
    sysIdTurnActiveOut.set(value);
  }

  @Override
  public void setSysIdTurnLastCompleted(double value) {
    sysIdTurnLastCompletedOut.set(value);
  }

  @Override
  public void setSysIdTurnLastCompletedPhase(String value) {
    sysIdTurnLastCompletedPhaseOut.set(value == null ? "" : value);
  }

  @Override
  public void setVisionStatus(String value) {
    visionStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setLogRollStatus(String value) {
    logRollStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setLogRollLastTimestamp(double value) {
    logRollLastTimestampOut.set(value);
  }

  @Override
  public void setLogRollCount(int value) {
    logRollCountOut.set(value);
  }

  @Override
  public void setLogCleanStatus(String value) {
    logCleanStatusOut.set(value == null ? "" : value);
  }

  @Override
  public void setLogCleanLastTimestamp(double value) {
    logCleanLastTimestampOut.set(value);
  }

  @Override
  public void setLogCleanCount(int value) {
    logCleanCountOut.set(value);
  }

  @Override
  public void setLogCleanDeletedEntries(int value) {
    logCleanDeletedEntriesOut.set(value);
  }
}
