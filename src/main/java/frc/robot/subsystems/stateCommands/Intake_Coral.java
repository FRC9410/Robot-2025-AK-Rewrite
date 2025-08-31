// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stateCommands;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

/** Add your docs here. */
public class Intake_Coral {
    HopperSubsystem hopperSubsystem = new HopperSubsystem();
    static private final double voltage = Constants.HopperConstants.START_VOLTAGE;
    public void execute() {
        hopperSubsystem.setVoltage((double) voltage);
    }

    public void exit() {
        hopperSubsystem.setVoltage(Constants.HopperConstants.STOP_VOLTAGE);
    }
}
