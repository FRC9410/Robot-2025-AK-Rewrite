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
      HopperSubsystem hopper, 
      AlgaeWristSubsystem algaeWrist, 
      AlgaeIntakeSubsystem algaeIntake, 
      SensorsSubsystem sensorsSubsystem, 
      ElevatorSubsystem elevatorSubsystem, 
      EndEffectorSubsystem endEffectorSubsystem) {
    this.hopperSubsystem = hopper;
    this.algaeWristSubsystem = algaeWrist;
    this.algaeIntakeSubsystem = algaeIntake;
    this.sensorsSubsystem = sensorsSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
  }

  RobotState INIT_STATE = RobotState.READY_STATE;
  RobotState READY_STATE = RobotState.READY_STATE;

  private boolean hopperMotorsRunning = false;
  private boolean endEffectorMotorsRunning = false;
  private boolean hasAlgae = false;
  private boolean shouldHoldAlgae = false; 
  private boolean shouldReturnToReadyStateFromHoldingAlgae = false;


  public enum RobotState {
    READY_STATE, // This is the default state when the robot is not doing anything
    ALGAE_READY_STATE,
    CLIMB,
    INTAKE_CORAL,
    INTAKE_ALGAE_LOWER,
    INTAKE_ALGAE_UPPER,
    INTAKE_ALGAE_GROUND,
    HOLD_ALGAE,
    SCORE_ALGAE_PROCESSOR,
    SCORE_ALGAE_BARGE,
    CORAL_SCORE_L1_LEFT,
    CORAL_SCORE_L2_LEFT,
    CORAL_SCORE_L3_LEFT,
    CORAL_SCORE_L4_LEFT,
    CORAL_SCORE_L1_RIGHT,
    CORAL_SCORE_L2_RIGHT,
    CORAL_SCORE_L3_RIGHT,
    CORAL_SCORE_L4_RIGHT,
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
      
      if(this.wantedRobotState == RobotState.HOLD_ALGAE || this.currentRobotState == RobotState.SCORE_ALGAE_PROCESSOR || this.currentRobotState == RobotState.SCORE_ALGAE_BARGE) { //I know this is messy
        this.currentRobotState = RobotState.ALGAE_READY_STATE;
        if (hopperSubsystem.isReady() && endEffectorSubsystem.isReady() && elevatorSubsystem.isReady()) {
          this.currentRobotState = this.wantedRobotState;
        }
      } else {
        this.currentRobotState = READY_STATE;
        if (isReadyState()) {
          this.currentRobotState = this.wantedRobotState;
        }
      }
    }

    switch (this.currentRobotState) {
      case READY_STATE:
        executeReadyState();
        break;
      case ALGAE_READY_STATE:
        executeAlgaeReady();
        break;
      case CLIMB:
        // Climb logic here
        break;
      case INTAKE_CORAL:
        executeIntakeCoral();
        break;
      case INTAKE_ALGAE_LOWER:
        // Intake algae lower logic here
        break;
      case INTAKE_ALGAE_UPPER:
        // Intake algae upper logic here
        break;
      case INTAKE_ALGAE_GROUND:
        // Intake algae ground logic here
        break;
      case HOLD_ALGAE:
        // Hold algae logic here
        break;
      case SCORE_ALGAE_PROCESSOR:
        // Score algae processor logic here
        break;
      case SCORE_ALGAE_BARGE: 
        break;
      case CORAL_SCORE_L1_LEFT:
        executeScoreCoralL1Left();
        break;
      case CORAL_SCORE_L2_LEFT:
        // Score coral L2 left logic here
        break;
      case CORAL_SCORE_L3_LEFT:
        // Score coral L3 left logic here
        break;
      case CORAL_SCORE_L4_LEFT:
        // Score coral L4 left logic here
        break;
      case CORAL_SCORE_L1_RIGHT:
        // Score coral L1 right logic here
        break;
      case CORAL_SCORE_L2_RIGHT:
        // Score coral L2 right logic here
        break;
      case CORAL_SCORE_L3_RIGHT:
        // Score coral L3 right logic here
        break;
      case CORAL_SCORE_L4_RIGHT:
        // Score coral L4 right logic here
        break;
    }
  }

  private boolean isReadyState() {
    if (elevatorSubsystem.isReady()
     && hopperSubsystem.isReady() 
     && endEffectorSubsystem.isReady() 
     && algaeWristSubsystem.isAtHomePosition()
     && algaeIntakeSubsystem.isReady())  { // check subsystems for readiness
      return true;
    }
    return false;
  }



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 
  /// State Execution Methods
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

  private void executeAlgaeReady() {
    if (hopperMotorsRunning) { // stop hopper motors
      hopperSubsystem.stopMotors();
      hopperMotorsRunning = false;
    }
    if (endEffectorMotorsRunning) { // stop end effector motors
      endEffectorSubsystem.stopMotors();
      endEffectorMotorsRunning = false;
    }
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.HOME_POSITION)) { // move elevator to home if not already there
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.HOME_POSITION);
    } 
  }
  

  private void executeReadyState() {
    if (hopperMotorsRunning) { // stop hopper motors
      hopperSubsystem.stopMotors();
      hopperMotorsRunning = false;
    }
    if (endEffectorMotorsRunning) { // stop end effector motors
      endEffectorSubsystem.stopMotors();
      endEffectorMotorsRunning = false;
    }
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.HOME_POSITION)) { // move elevator to home if not already there
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.HOME_POSITION);
      shouldHoldAlgae = false;
      shouldReturnToReadyStateFromHoldingAlgae = false;
    }
    if (!algaeWristSubsystem.isAtHomePosition()) { // move algae wrist to home if not already there
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE);
    }
    if (algaeIntakeSubsystem.isRunning()) { // stop algae intake if running
      algaeIntakeSubsystem.stopIntake();
    }
  }


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
      setWantedState(RobotState.READY_STATE); // failsafe, should never occur.
    }
  } 

  private void executeHoldAlgae() {
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.HOME_POSITION)) {
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.HOME_POSITION); // move elevator to height if its not already there
      shouldHoldAlgae = false;
      shouldReturnToReadyStateFromHoldingAlgae = true;
    } else {
      if (!algaeWristSubsystem.isAtDownPosition()) {
        algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE); // move wrist down
      }
      if (!algaeIntakeSubsystem.isRunning()) {
        algaeIntakeSubsystem.stallIntake(); // start algae intake
      }
    }
  }

  private void executeIntakeAlgaeLower() { 
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION)) {
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION); // move elevator to height if its not already there
      shouldHoldAlgae = true;
    } else {
      if (!algaeWristSubsystem.isAtDownPosition()) {
        algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE); // move wrist down
      } else {
        if (!algaeIntakeSubsystem.isRunning()) {
          algaeIntakeSubsystem.intakeAlgae(); // start algae intake
        }
      }
    }
  }


  private void executeScoreCoralL1Left() {
    if (sensorsSubsystem.hasPiece()) {
      // need to implement position logic here
        if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L1_SCORE_POSITION)) { // move elevator to height if its not already there
          elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_SCORE_POSITION); 
        } else {  // if elevator is at height, outtake coral
          endEffectorSubsystem.outtakeCoral();
        }
    } else { // No piece, intake coral instead. 
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

  public boolean getAlgaeShouldGoDown() {
    return shouldHoldAlgae;
  }

  public boolean getShouldReturnToReadyStateFromHoldingAlgae() {
    return shouldReturnToReadyStateFromHoldingAlgae;
  }
}
