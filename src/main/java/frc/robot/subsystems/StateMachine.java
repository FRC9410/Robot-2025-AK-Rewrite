// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StateMachine extends SubsystemBase {
  HopperSubsystem hopperSubsystem;
  AlgaeWristSubsystem algaeWristSubsystem;
  AlgaeIntakeSubsystem algaeIntakeSubsystem;
  SensorsSubsystem sensorsSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;
  ClimberSubsystem climberSubsystem;
  // Dashboard dashboard;

  public StateMachine(
      HopperSubsystem hopper,
      AlgaeWristSubsystem algaeWrist,
      AlgaeIntakeSubsystem algaeIntake,
      SensorsSubsystem sensorsSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      EndEffectorSubsystem endEffectorSubsystem,
      ClimberSubsystem climberSubsystem) {
    this.hopperSubsystem = hopper;
    this.algaeWristSubsystem = algaeWrist;
    this.algaeIntakeSubsystem = algaeIntake;
    this.sensorsSubsystem = sensorsSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.climberSubsystem = climberSubsystem;
  }

  RobotState INIT_STATE = RobotState.READY_STATE;
  RobotState READY_STATE = RobotState.READY_STATE;

  private boolean hopperMotorsRunning = false;
  private boolean endEffectorMotorsRunning = false;
  private boolean hasAlgae = false;
  private boolean shouldHoldAlgae = false;
  private boolean shouldReturnToReadyStateFromHoldingAlgae = false;

  public enum CoralPositions {
    NO_POSITION,
    LEFT_L1,
    LEFT_L2,
    LEFT_L3,
    LEFT_L4,
    RIGHT_L1,
    RIGHT_L2,
    RIGHT_L3,
    RIGHT_L4
  }

  public enum RobotState {
    READY_STATE, // This is the default state when the robot is not doing anything
    CLIMB,
    INTAKE_CORAL,
    DESCORE_ALGAE_LOWER,
    DESCORE_ALGAE_UPPER,
    SCORAL // score + coral = scoral. -aiden
  }

  /*
   * Algae gone 2
   * algae gone 3
   * score 1
   * score 2
   * score 3
   * score 4
   * climb
   * intake coral
   *
   *
   */

  private CoralPositions selectedCoralPosition = CoralPositions.NO_POSITION;

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
      case READY_STATE:
        executeReadyState();
        break;
      case CLIMB:
        executeClimbState();
        break;
      case INTAKE_CORAL:
        executeIntakeCoral();
        break;
      case SCORAL:
        executeScoreCoral(selectedCoralPosition);
        break;
    }
  }

  private boolean isReadyState() {
    if (elevatorSubsystem.isReady()
        && hopperSubsystem.isReady()
        && endEffectorSubsystem.isReady()
        && algaeWristSubsystem.isAtHomePosition()
        && algaeIntakeSubsystem.isReady()
        && climberSubsystem.isReady()) { // check subsystems for readiness
      return true;
    }
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// State Execution Methods
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  private void executeReadyState() {
    if (hopperMotorsRunning) { // stop hopper motors
      hopperSubsystem.stopMotors();
      hopperMotorsRunning = false;
    }
    if (endEffectorMotorsRunning) { // stop end effector motors
      endEffectorSubsystem.stopMotors();
      endEffectorMotorsRunning = false;
    }
    if (!elevatorSubsystem.isAtPosition(
        Constants.ElevatorConstants.HOME_POSITION)) { // move elevator to home if not already there
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
    if (!climberSubsystem.isReady()) { // stop climber if running
      climberSubsystem.stop();
    }
  }

  private void executeClimbState() {}

  private void executeIntakeCoral() {
    if (!sensorsSubsystem.hasPiece()) { // if no piece
      if (!hopperMotorsRunning) { // turn on hopper
        hopperSubsystem.startMotors();
        hopperMotorsRunning = true;
      }
      if (sensorsSubsystem.isIntakeLaserBroken()) { // if intake beam broken
        if (!endEffectorMotorsRunning) { // turn on end effector
          endEffectorSubsystem.intakeCoral();
          endEffectorMotorsRunning = true;
        }
      } else { // if intake beam is not broken
        if (endEffectorMotorsRunning) { // stop end effector
          endEffectorSubsystem.stopMotors();
          endEffectorMotorsRunning = false;
        }
      }

    } else { // there is a piece
      if (hopperMotorsRunning) { // stop hopper motors
        hopperSubsystem.stopMotors();
      }
      setWantedState(RobotState.READY_STATE); // failsafe, should never occur.
    }
  }

  private void executeScoreCoral(CoralPositions coralPosition) {
    double elevatorPosition;
    double xOffset; // I'm not sure if this is right but just to get the idea
    switch (coralPosition) {
      case LEFT_L1:
        elevatorPosition = Constants.ElevatorConstants.L1_SCORE_POSITION;
        break;
      case LEFT_L2:
        elevatorPosition = Constants.ElevatorConstants.L2_SCORE_POSITION;
        break;
      case LEFT_L3:
        elevatorPosition = Constants.ElevatorConstants.L3_SCORE_POSITION;
        break;
      case LEFT_L4:
        elevatorPosition = Constants.ElevatorConstants.L4_SCORE_POSITION;
        break;
      case RIGHT_L1:
        elevatorPosition = Constants.ElevatorConstants.L1_SCORE_POSITION;
        break;
      case RIGHT_L2:
        elevatorPosition = Constants.ElevatorConstants.L2_SCORE_POSITION;
        break;
      case RIGHT_L3:
        elevatorPosition = Constants.ElevatorConstants.L3_SCORE_POSITION;
        break;
      case RIGHT_L4:
        elevatorPosition = Constants.ElevatorConstants.L4_SCORE_POSITION;
        break;
      default:
        elevatorPosition = Constants.ElevatorConstants.HOME_POSITION;
        break;
    }

    if (sensorsSubsystem.hasPiece()) {
      // need to implement position logic here
      if (!elevatorSubsystem.isAtPosition(
          elevatorPosition)) { // move elevator to height if its not already there
        elevatorSubsystem.setPosition(elevatorPosition);
      } else { // if elevator is at height, outtake coral
        endEffectorSubsystem.outtakeCoral();
      }
    } else { // No piece, intake coral instead.
      if (!elevatorSubsystem.isAtPosition(
          Constants.ElevatorConstants.HOME_POSITION)) { // if no piece and elevator not home
        elevatorSubsystem.setPosition(
            Constants.ElevatorConstants.HOME_POSITION); // move elevator to home
        endEffectorSubsystem.setVoltage(
            Constants.EndEffectorConstants.STOP_VOLTAGE); // turn off end effector
      }
      setWantedState(RobotState.INTAKE_CORAL);
    }
  }

  public Command setWantedState(RobotState state) {
    return runOnce(() -> wantedRobotState = state);
  }

  public void removeAlgaeUpper() {
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION)) {
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION);
    }
    if (!algaeWristSubsystem.isAtDownPosition()) {
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE);
    }
    if (!algaeIntakeSubsystem.isRunning()) {
      algaeIntakeSubsystem.outtakeAlgae();
    }
  }

  public void removeAlgaeLower() {
    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L2_ALGAE_POSITION)) {
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.L2_ALGAE_POSITION);
    }
    if (!algaeWristSubsystem.isAtDownPosition()) {
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE);
    }
    if (!algaeIntakeSubsystem.isRunning()) {
      algaeIntakeSubsystem.outtakeAlgae();
    }
  }

  public CoralPositions getSelectedCoralPosition() {
    return selectedCoralPosition;
  }

  public void setSelectedCoralPosition(CoralPositions position) {
    selectedCoralPosition = position;
  }
}
