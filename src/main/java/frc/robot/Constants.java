// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CanBusConstants {
    public static final String CANIVORE_BUS = "canivore";
  }

  public static final class HopperConstants {
    public static final int PRIMARY_CAN_ID = 32;
    public static final int SECONDARY_CAN_ID = 31;
    // Hopper speed constant (placeholder value; adjust as needed)
    public static final double START_VOLTAGE = 3.5;
    public static final double STOP_VOLTAGE = 0.0;
  }

  public static final class MapConstants {
    public static final String ELEVATOR_POSITION = "elevatorPosition";
    public static final String END_EFFECTOR_VOLTAGE = "endEffectorVoltage";
    public static final String HOPPER_VOLTAGE = "hopperVoltage";
    public static final String CLIMBER_POSITION = "climberPosition";
    public static final String WRIST_POSITION = "wristPosition";
    public static final String INTAKE_VOLTAGE = "intakeSpeed";
    public static final String POSE = "pose";
    public static final String TARGET_ROTATION = "targetRotation";
    public static final String TARGET_POSE = "targetPose";
    public static final String HAS_PIECE = "hasPiece";
    public static final String AUTO = "auto";
  }
}
