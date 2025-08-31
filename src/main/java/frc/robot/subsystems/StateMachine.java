// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class StateMachine extends SubsystemBase {

  /** Creates a new StateMachine. */
  public StateMachine(
    HopperSubsystem hopperSubsystem
  ) {}

  public enum WantedRobotState {
    HOME,
    STOPPED,
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
    HOME,
    DEFAULT_STATE,
    STOPPED,
    CLIMB,
    INTAKE_CORAL,
    INTAKE_ALGAE_LOWER,
    INTAKE_ALGAE_UPPER,
    INTAKE_GROUND,
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

  private WantedRobotState wantedRobotState = WantedRobotState.STOPPED;
  private CurrentRobotState currentRobotState = CurrentRobotState.STOPPED;
  private CurrentRobotState previousRobotState;

  @Override
  public void periodic() {
    currentRobotState = handleRobotStateTransitions();
    applyStates();
  }

  private stop(enum previousState) {
    switch (previousState) {
      case Intake_CORAL:
        intake_coral.exit();
        break;

    }
  }
  //This method handles the exit of a previous state, like intake_coral.exit() turning off the motors.

  private CurrentRobotState handleRobotStateTransitions() {
    previousRobotState = currentRobotState;



    switch (wantedRobotState) {
      default:
        stop(previousRobotState)
        currentRobotState = CurrentRobotState.STOPPED;
        break;
      case HOME:
        currentRobotState = CurrentRobotState.HOME;
        break;
      case DEFAULT_STATE:
        currentRobotState = CurrentRobotState.STOPPED;
        break;
      case CLIMB:
        currentRobotState = CurrentRobotState.CLIMB;
        break;
      case INTAKE_CORAL:
        currentRobotState = CurrentRobotState.INTAKE_CORAL;
        break;
      case INTAKE_ALGAE_LOWER:
        currentRobotState = CurrentRobotState.INTAKE_ALGAE_LOWER;
        break;
      case INTAKE_ALGAE_UPPER:
        currentRobotState = CurrentRobotState.INTAKE_ALGAE_UPPER;
        break;
      case INTAKE_ALGAE_GROUND:
        currentRobotState = CurrentRobotState.INTAKE_GROUND;
        break;
      case SCORE_ALGAE_PROCESSOR:
        currentRobotState = CurrentRobotState.SCORE_ALGAE_PROCESSOR;
        break;
      case CORAL_SCORE_L1_LEFT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L1_LEFT;
        break;
      case CORAL_SCORE_L2_LEFT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L2_LEFT;
        break;
      case CORAL_SCORE_L3_LEFT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L3_LEFT;
        break;
      case CORAL_SCORE_L4_LEFT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L4_LEFT;
        break;
      case CORAL_SCORE_L1_RIGHT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L1_RIGHT;
        break;
      case CORAL_SCORE_L2_RIGHT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L2_RIGHT;
        break;
      case CORAL_SCORE_L3_RIGHT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L3_RIGHT;
        break;
      case CORAL_SCORE_L4_RIGHT:
        currentRobotState = CurrentRobotState.CORAL_SCORE_L4_RIGHT;
        break;
    }
    return (currentRobotState);
  }

  private void applyStates() {
    // Put the logic for conditional state maneuvers here (save current status in bools etc for
    // conditional logic)

    switch (currentRobotState) {
      case HOME:

        break;
      case DEFAULT_STATE:

        break;
      case STOPPED:
;
        break;
      case CLIMB:

        break;
      case INTAKE_CORAL:
        Intake_Coral.execute();
        break;
      case INTAKE_ALGAE_LOWER:

        break;
      case INTAKE_ALGAE_UPPER:

        break;
      case INTAKE_GROUND:

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
    }
  }


  public void setWantedRobotState(WantedRobotState wantedRobotState) {
    this.wantedRobotState = wantedRobotState;
    handleRobotStateTransitions();
  }

  public WantedRobotState getWantedRobotState() {
    return wantedRobotState;
  }

  public CurrentRobotState getCurrentRobotState() {
    return currentRobotState;
  }

  public CurrentRobotState getPreviousRobotState() {
    return previousRobotState;
  }

  public Command setStateCommand(WantedRobotState superState) {
    return setStateCommand(superState, false);
  }

  public Command setStateCommand(WantedRobotState superState, boolean runIfClimberDeployed) {
    Command commandToReturn = new InstantCommand(() -> setWantedRobotState(superState));
    // safeguard anti-move-while-climb logic here
    return commandToReturn;
  }
}
