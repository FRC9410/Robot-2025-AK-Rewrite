// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.*;

public class StateMachine extends SubsystemBase {
  HopperSubsystem hopperSubsystem;
  AlgaeWristSubsystem algaeWristSubsystem;
  AlgaeIntakeSubsystem algaeIntakeSubsystem;
  SensorsSubsystem sensorsSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;
  ClimberSubsystem climberSubsystem;
  RobotContainer robotContainer;
  Drive drive;
  // Dashboard dashboard;

  public StateMachine(
      HopperSubsystem hopper,
      AlgaeWristSubsystem algaeWrist,
      AlgaeIntakeSubsystem algaeIntake,
      SensorsSubsystem sensorsSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      EndEffectorSubsystem endEffectorSubsystem,
      ClimberSubsystem climberSubsystem,
      RobotContainer robotContainer) {
    this.hopperSubsystem = hopper;
    this.algaeWristSubsystem = algaeWrist;
    this.algaeIntakeSubsystem = algaeIntake;
    this.sensorsSubsystem = sensorsSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
  }

  RobotState INIT_STATE = RobotState.READY_STATE;
  RobotState READY_STATE = RobotState.READY_STATE;

  private boolean hopperMotorsRunning = false;
  private boolean endEffectorMotorsRunning = false;
  private boolean hasAlgae = false;
  private boolean shouldHoldAlgae = false;
  private boolean shouldReturnToReadyStateFromHoldingAlgae = false;

  public static enum CoralPositions {
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

  public static enum RobotState {
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
    boolean isBlueAlliance = true;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        isBlueAlliance = false;
      }
    }

    if (this.currentRobotState != this.wantedRobotState) {
      this.currentRobotState = READY_STATE;
      if (true) { // isReadyState()) ????? Works fine without it? -aidan. 
        this.currentRobotState = this.wantedRobotState;
      }
    }

    System.out.println(currentRobotState.toString());
    System.out.println(selectedCoralPosition.toString());
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
      case DESCORE_ALGAE_LOWER:
        removeAlgaeLower(isBlueAlliance);
        break;
      case DESCORE_ALGAE_UPPER:
        removeAlgaeUpper(isBlueAlliance);
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
      endEffectorSubsystem.setVelocity(0);
      endEffectorMotorsRunning = false;
    }
    if (!elevatorSubsystem.isAtPosition(
        Constants.ElevatorConstants.HOME_POSITION)) { // move elevator to home if not already there
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.HOME_POSITION);
      shouldHoldAlgae = false;
      shouldReturnToReadyStateFromHoldingAlgae = false;
    }
    if (!algaeWristSubsystem.isAtHomePosition()) { // move algae wrist to home if not already there
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.UP_POSITION);
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
          endEffectorSubsystem.setVelocity(7);
          endEffectorMotorsRunning = true;
        }
      } else { // if intake beam is not broken
        if (endEffectorMotorsRunning) { // stop end effector
          endEffectorSubsystem.setVelocity(0);
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
      System.out.println("Has piece");
      // need to implement position logic here
      if (!elevatorSubsystem.isAtPosition(
          elevatorPosition)) { // move elevator to height if its not already there
        elevatorSubsystem.setPosition(elevatorPosition);
      } else { // if elevator is at height, outtake coral
        endEffectorSubsystem.outtakeCoral();
      }
    } else { // No piece, intake coral instead.
      System.out.println("Doesn't have piece");
      if (!elevatorSubsystem.isAtPosition(
          Constants.ElevatorConstants.HOME_POSITION)) { // if no piece and elevator not home
        elevatorSubsystem.setPosition(
            Constants.ElevatorConstants.HOME_POSITION); // move elevator to home
        endEffectorSubsystem.setVoltage(
            Constants.EndEffectorConstants.STOP_VOLTAGE); // turn off end effector
        System.out.println("True");
      }
      setWantedState(RobotState.INTAKE_CORAL);
      System.out.println("False");
    }
  }

  public void setWantedState(RobotState state) {
    wantedRobotState = state;
  }

  public void removeAlgaeUpper(Boolean isBlueAlliance) {
    List<Translation2d> polygonToUse = null;
    if (isBlueAlliance) {
      polygonToUse = Constants.bluePolygon;
    } else {
      polygonToUse = Constants.redPolygon;
    }

    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L2_ALGAE_POSITION)) {
      System.out.println("Moving to L1 algae position");
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.L2_ALGAE_POSITION);
    }
    final boolean inside = GeometryUtil.pointInPolygon(polygonToUse, drive.getPose().getTranslation());
    if (!algaeWristSubsystem.isAtDownPosition() && !inside) {
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.DOWN_POSITION);
    } else {
      if (!algaeWristSubsystem.isAtHomePosition() && inside) {
        algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.UP_POSITION);
      }
    }
    if (!algaeIntakeSubsystem.isRunning()) {
      algaeIntakeSubsystem.outtakeAlgae();
    }
  }

  public void removeAlgaeLower(boolean isBlueAlliance) {
    List<Translation2d> polygonToUse = null;
    if (isBlueAlliance) {
      polygonToUse = Constants.bluePolygon;
    } else {
      polygonToUse = Constants.redPolygon;
    }

    if (!elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION)) {
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_ALGAE_POSITION);
    }
    final boolean inside = GeometryUtil.pointInPolygon(polygonToUse, drive.getPose().getTranslation());
    if (!algaeWristSubsystem.isAtDownPosition() && !inside) {
      algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.DOWN_POSITION);
    } else {
      if (!algaeWristSubsystem.isAtHomePosition() && inside) {
        algaeWristSubsystem.setPosition(Constants.AlgaeWristConstants.UP_POSITION);
      }
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
    setWantedState(RobotState.SCORAL);
  }

  public RobotState getCurrentRobotState() {
    return currentRobotState;
  }

  public CoralPositions getCurrentCoralPosition() {
    return selectedCoralPosition;
  }
}
