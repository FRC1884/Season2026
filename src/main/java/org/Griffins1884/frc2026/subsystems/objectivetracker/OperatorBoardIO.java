// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.Griffins1884.frc2026.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface OperatorBoardIO {
  @AutoLog
  class OperatorBoardIOInputs {
    public String[] requestedState = new String[] {}; // Superstructure.SuperState name
  }

  default void updateInputs(OperatorBoardIOInputs inputs) {}

  default void setRequestedState(String value) {}

  default void setCurrentState(String value) {}

  default void setRequestAccepted(boolean value) {}

  default void setRequestReason(String value) {}

  default void setClimbPhase(String value) {}

  default void setClimbLevel(int value) {}

  default void setTargetType(String value) {}

  default void setTargetPose(double[] value) {}

  default void setTargetPoseValid(boolean value) {}

  default void setRobotPose(double[] value) {}

  default void setHasBall(boolean value) {}

  default void setDsMode(String value) {}

  default void setBatteryVoltage(double value) {}

  default void setBrownout(boolean value) {}

  default void setAlliance(String value) {}

  default void setMatchTime(double value) {}

  default void setTurretAtSetpoint(boolean value) {}

  default void setTurretMode(String value) {}

  default void setVisionStatus(String value) {}
}
