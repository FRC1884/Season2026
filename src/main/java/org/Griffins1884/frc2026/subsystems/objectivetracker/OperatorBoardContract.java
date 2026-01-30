package org.Griffins1884.frc2026.subsystems.objectivetracker;

final class OperatorBoardContract {
  static final String BASE = "/OperatorBoard/v1";
  static final String TO_ROBOT = BASE + "/ToRobot";
  static final String TO_DASHBOARD = BASE + "/ToDashboard";

  static final String REQUESTED_STATE = "RequestedState";
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
  static final String TURRET_AT_SETPOINT = "TurretAtSetpoint";
  static final String TURRET_MODE = "TurretMode";
  static final String VISION_STATUS = "VisionStatus";

  private OperatorBoardContract() {}
}
