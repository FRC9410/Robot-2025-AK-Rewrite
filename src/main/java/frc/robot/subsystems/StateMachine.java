// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.net.http.HttpClient.Redirect;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class StateMachine extends SubsystemBase {
  HopperSubsystem hopperSubsystem;
  AlgaeWristSubsystem algaeWristSubsystem;
  AlgaeIntakeSubsystem algaeIntakeSubsystem;
  SensorsSubsystem sensorsSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;

  public StateMachine(
      HopperSubsystem hopper, AlgaeWristSubsystem algaeWrist, AlgaeIntakeSubsystem algaeIntake, SensorsSubsystem sensorsSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.hopperSubsystem = hopper;
    this.algaeWristSubsystem = algaeWrist;
    this.algaeIntakeSubsystem = algaeIntake;
    this.sensorsSubsystem = sensorsSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
  }

  RobotState INIT_STATE = RobotState.DEFAULT_STATE;
  RobotState READY_STATE = RobotState.DEFAULT_STATE;

  private boolean hopperMotorsRunning = false;
  private boolean endEffectorMotorsRunning = false;


  public enum RobotState {
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

  private RobotState wantedRobotState = INIT_STATE;
  private RobotState currentRobotState = INIT_STATE;
  private RobotState previousRobotState;

  @Override
  public void periodic() {
    handleRobotStateTransitions();
  }

  private void handleRobotStateTransitions() {
    if (this.currentRobotState != this.wantedRobotState) {
      this.currentRobotState = READY_STATE;

      if (isReadyState()) {
        this.currentRobotState = this.wantedRobotState;
      }
    }

    switch (this.currentRobotState) {
      case INTAKE_CORAL:
        // executeIntakeCoral();
        break;
      case CORAL_SCORE_L1_LEFT:
        executeScoreCoralL1Left();
        break;
    }
  }

  private boolean isReadyState() {
    // if () { // check subsystems for readiness
    //   return true
    // }
    // return false\
    return true;
  }

  ////////////////////////////////////////////////////////////
  /// 
  /// State Execution Methods
  ///
  ////////////////////////////////////////////////////////////
  
  private void executeIntakeCoral() {
    if (!sensorsSubsystem.hasPiece()) { // if no piece
      if (!hopperMotorsRunning) { // turn on hopper
        hopperSubsystem.startMotors();
        hopperMotorsRunning = true;
      }
      if(sensorsSubsystem.isIntakeLaserBroken()) { // if intake beam broken
        if (!endEffectorMotorsRunning) { // turn on end effector
          endEffectorSubsystem.intakeCoral();
          endEffectorMotorsRunning = true;
        }
      } else {                    // if intake beam is not broken
        if (endEffectorMotorsRunning) { // stop end effector
          endEffectorSubsystem.stopMotors();
          endEffectorMotorsRunning = false;
        }
      }

    } else { // there is a piece
      if(hopperMotorsRunning) { // stop hopper motors
       hopperSubsystem.stopMotors();
      }
      setWantedState(RobotState.DEFAULT_STATE);
    }
  } 

  private void executeScoreCoralL1Left() {
    if (sensorsSubsystem.hasPiece()) {
      
    } else {
      if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.HOME_POSITION)) { // if no piece and elevator not home 
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.HOME_POSITION); // move elevator to home
        endEffectorSubsystem.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE); // turn off end effector
      }
      setWantedState(RobotState.INTAKE_CORAL);
    }
  }

  public Command setWantedState(RobotState state) {
    return runOnce(() -> wantedRobotState = state);
  }
}
