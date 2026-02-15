package org.Griffins1884.frc2026.subsystems.objectivetracker;

final class OperatorBoardContract {
  static final String BASE = "/OperatorBoard/v1";
  static final String TO_ROBOT = BASE + "/ToRobot";
  static final String TO_DASHBOARD = BASE + "/ToDashboard";

  static final String REQUESTED_STATE = "RequestedState";
  static final String AUTO_STATE_ENABLE = "AutoStateEnable";
  static final String PLAY_SWERVE_MUSIC = "PlaySwerveMusic";
  static final String STOP_SWERVE_MUSIC = "StopSwerveMusic";
  static final String SWERVE_MUSIC_VOLUME = "SwerveMusicVolume";
  static final String ROLL_LOGS = "RollLogs";
  static final String SYSID_DRIVE_PHASE = "SysIdDrivePhase";
  static final String SYSID_DRIVE_ACTIVE = "SysIdDriveActive";
  static final String SYSID_DRIVE_LAST_COMPLETED = "SysIdDriveLastCompleted";
  static final String SYSID_TURN_PHASE = "SysIdTurnPhase";
  static final String SYSID_TURN_ACTIVE = "SysIdTurnActive";
  static final String SYSID_TURN_LAST_COMPLETED = "SysIdTurnLastCompleted";
  static final String CURRENT_STATE = "CurrentState";
  static final String REQUEST_ACCEPTED = "RequestAccepted";
  static final String REQUEST_REASON = "RequestReason";
  static final String CLIMB_PHASE = "ClimbPhase";
  static final String CLIMB_LEVEL = "ClimbLevel";
  static final String TARGET_TYPE = "TargetType";
  static final String TARGET_POSE = "TargetPose";
  static final String TARGET_POSE_VALID = "TargetPoseValid";
  static final String ROBOT_POSE = "RobotPose";
  static final String HAS_BALL = "HasBall";
  static final String DS_MODE = "DsMode";
  static final String BATTERY_VOLTAGE = "BatteryVoltage";
  static final String BROWNOUT = "Brownout";
  static final String ALLIANCE = "Alliance";
  static final String MATCH_TIME = "MatchTime";
  static final String HUB_TIMEFRAME = "HubTimeframe";
  static final String HUB_STATUS_VALID = "HubStatusValid";
  static final String RED_HUB_STATUS = "RedHubStatus";
  static final String BLUE_HUB_STATUS = "BlueHubStatus";
  static final String OUR_HUB_STATUS = "OurHubStatus";
  static final String OUR_HUB_ACTIVE = "OurHubActive";
  static final String AUTO_WINNER_ALLIANCE = "AutoWinnerAlliance";
  static final String GAME_DATA_RAW = "GameDataRaw";
  static final String HUB_RECOMMENDATION = "HubRecommendation";
  static final String TURRET_AT_SETPOINT = "TurretAtSetpoint";
  static final String TURRET_MODE = "TurretMode";
  static final String VISION_STATUS = "VisionStatus";

  private OperatorBoardContract() {}
}
