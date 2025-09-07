// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AlgaeWristConstants;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;


public class StateMachine extends SubsystemBase {
  HopperSubsystem hopperSubsystem;
  AlgaeWristSubsystem algaeWristSubsystem;
  AlgaeIntakeSubsystem algaeIntakeSubsystem;

  public StateMachine(HopperSubsystem hopper, AlgaeWristSubsystem algaeWrist, AlgaeIntakeSubsystem algaeIntake) {
    this.hopperSubsystem = hopper;
    this.algaeWristSubsystem = algaeWrist;
    this.algaeIntakeSubsystem = algaeIntake;
  }

  public enum WantedRobotState {
    DEFAULT_STATE, // This is the default state when the robot is not doing anything
    CLIMB,
    INTAKE_CORAL,
    INTAKE_ALGAE_LOWER,
    INTAKE_ALGAE_UPPER,
    INTAKE_ALGAE_GROUND,
    SCORE_ALGAE_PROCESSOR,
    CORAL_SCORE_L1_LEFT,
    CORAL_SCORE_L2_LEFT,
    CORAL_SCORE_L3_LEFT,
    CORAL_SCORE_L4_LEFT,
    CORAL_SCORE_L1_RIGHT,
    CORAL_SCORE_L2_RIGHT,
    CORAL_SCORE_L3_RIGHT,
    CORAL_SCORE_L4_RIGHT
  }

  public enum CurrentRobotState {
    DEFAULT_STATE, // This is the default state when the robot is not doing anything
    CLIMB,
    INTAKE_CORAL,
    INTAKE_ALGAE_LOWER,
    INTAKE_ALGAE_UPPER,
    INTAKE_ALGAE_GROUND,
    SCORE_ALGAE_PROCESSOR,
    CORAL_SCORE_L1_LEFT,
    CORAL_SCORE_L2_LEFT,
    CORAL_SCORE_L3_LEFT,
    CORAL_SCORE_L4_LEFT,
    CORAL_SCORE_L1_RIGHT,
    CORAL_SCORE_L2_RIGHT,
    CORAL_SCORE_L3_RIGHT,
    CORAL_SCORE_L4_RIGHT
  }

  private WantedRobotState wantedRobotState = WantedRobotState.DEFAULT_STATE;
  private CurrentRobotState currentRobotState = CurrentRobotState.DEFAULT_STATE;
  private CurrentRobotState previousRobotState;


  @Override
  public void periodic() {
    handleRobotStateTransitions();
  }

  private CurrentRobotState wantedToCurrentState(WantedRobotState state) {
    switch (state) {
      case DEFAULT_STATE:
        return CurrentRobotState.DEFAULT_STATE;
      case CLIMB:
        return CurrentRobotState.CLIMB;
      case INTAKE_CORAL:
        return CurrentRobotState.INTAKE_CORAL;
      case INTAKE_ALGAE_LOWER:
        return CurrentRobotState.INTAKE_ALGAE_LOWER;
      case INTAKE_ALGAE_UPPER:
        return CurrentRobotState.INTAKE_ALGAE_UPPER;
      case INTAKE_ALGAE_GROUND:
        return CurrentRobotState.INTAKE_ALGAE_GROUND;
      case SCORE_ALGAE_PROCESSOR:
        return CurrentRobotState.SCORE_ALGAE_PROCESSOR;
      case CORAL_SCORE_L1_LEFT:
        return CurrentRobotState.CORAL_SCORE_L1_LEFT;
      case CORAL_SCORE_L2_LEFT:
        return CurrentRobotState.CORAL_SCORE_L2_LEFT;
      case CORAL_SCORE_L3_LEFT:
        return CurrentRobotState.CORAL_SCORE_L3_LEFT;
      case CORAL_SCORE_L4_LEFT:
        return CurrentRobotState.CORAL_SCORE_L4_LEFT;
      case CORAL_SCORE_L1_RIGHT:
        return CurrentRobotState.CORAL_SCORE_L1_RIGHT;
      case CORAL_SCORE_L2_RIGHT:
        return CurrentRobotState.CORAL_SCORE_L2_RIGHT;
      case CORAL_SCORE_L3_RIGHT:
        return CurrentRobotState.CORAL_SCORE_L3_RIGHT;
      case CORAL_SCORE_L4_RIGHT:
        return CurrentRobotState.CORAL_SCORE_L4_RIGHT;
      default:
        return CurrentRobotState.DEFAULT_STATE;
    } 
  }

  private void exitState(CurrentRobotState currentRobotState) {
    switch (currentRobotState) {
      case DEFAULT_STATE:
        break;
      case CLIMB:
        break;
      case INTAKE_CORAL:
        hopperSubsystem.exit();
        break;
      case INTAKE_ALGAE_LOWER:
        break;
      case INTAKE_ALGAE_UPPER:
        break;
      case INTAKE_ALGAE_GROUND:
        algaeWristSubsystem.setPosition(AlgaeWristConstants.MIN_POSITION); // ??
        algaeIntakeSubsystem.setVoltage(AlgaeIntakeConstants.STOP_VOLTAGE);
        break;
      case SCORE_ALGAE_PROCESSOR:
        break;
      case CORAL_SCORE_L1_LEFT:
        break;
      case CORAL_SCORE_L2_LEFT:
        break;
      case CORAL_SCORE_L3_LEFT:
        break;
      case CORAL_SCORE_L4_LEFT:
        break;
      case CORAL_SCORE_L1_RIGHT:
        break;
      case CORAL_SCORE_L2_RIGHT:
        break;
      case CORAL_SCORE_L3_RIGHT:
        break;
      case CORAL_SCORE_L4_RIGHT:
        break;
      default:
        break;
    }
  }

  private void enterState(CurrentRobotState currentRobotState) {
    switch (currentRobotState) {
      case DEFAULT_STATE:
        break;
      case CLIMB:
        break;
      case INTAKE_CORAL:
        hopperSubsystem.enter();
        break;
      case INTAKE_ALGAE_LOWER:
        break;
      case INTAKE_ALGAE_UPPER:
        break;
      case INTAKE_ALGAE_GROUND:
        algaeWristSubsystem.setPosition(AlgaeWristConstants.MIN_POSITION);
        algaeIntakeSubsystem.setVoltage(AlgaeIntakeConstants.INTAKE_VOLTAGE);
        break;
      case SCORE_ALGAE_PROCESSOR:
        break;
      case CORAL_SCORE_L1_LEFT:
        break;    
      case CORAL_SCORE_L2_LEFT:
        break;
      case CORAL_SCORE_L3_LEFT:
        break;
      case CORAL_SCORE_L4_LEFT:
        break;
      case CORAL_SCORE_L1_RIGHT:
        break;
      case CORAL_SCORE_L2_RIGHT:
        break;
      case CORAL_SCORE_L3_RIGHT:
        break;
      case CORAL_SCORE_L4_RIGHT:
        break;
      default:
        break;
    }
  }

  private void handleRobotStateTransitions() {
    if (currentRobotState != wantedToCurrentState(wantedRobotState)) {
      // Exit old state
      exitState(currentRobotState);
  
      // Update
      previousRobotState = currentRobotState;
      currentRobotState = wantedToCurrentState(wantedRobotState);
  
      // Enter new state
      enterState(currentRobotState);
    }
  }

  public Command setWantedState(WantedRobotState state) {
    return runOnce(() -> wantedRobotState = state);
  }
}
