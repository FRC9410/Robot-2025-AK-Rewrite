// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.CoralPositions;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Arrays;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_ANGULAR_RATE =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE =
      RotationsPerSecond.of(0.5)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_SPEED =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75;
  public double SKEW_COMPENSATION =
      0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final StateMachine stateMachine;
  private final boolean autoDrive;
  private final double STATIC_FRICTION_CONSTANT;
  private final PIDController driveToPointController;

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      StateMachine stateMachine,
      boolean autoDrive) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.stateMachine = stateMachine;
    this.autoDrive = autoDrive;
    this.STATIC_FRICTION_CONSTANT =
        0.170; // Adjust this value based on your robot's characteristics
    this.driveToPointController = new PIDController(2.5, 0, 0.1);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(STATIC_FRICTION_CONSTANT * drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE);
    final Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose = null;

    if (currentPose != null) {
      if (controller.a().getAsBoolean()) {
        targetPose = Constants.AutoConstants.TEST_POSITION;
      } else if (controller.y().getAsBoolean()) {
        targetPose = new Pose2d();
      } else if (controller.x().getAsBoolean()) {
        targetPose = Constants.AutoConstants.TEST_POSITION2;
      } else if (controller.b().getAsBoolean()) {
        targetPose = Constants.AutoConstants.TEST_POSITION3;
      }
      // getTargetPose(
      //     currentPose,
      //     stateMachine.getCurrentRobotState(),
      //     stateMachine.getCurrentCoralPosition());
    }

    if (currentPose != null && targetPose != null && autoDrive) {
      final Translation2d translationToPoint =
          currentPose.getTranslation().minus(targetPose.getTranslation());
      final double linearDistance = translationToPoint.getNorm();
      double ff = 0;
      if (linearDistance >= Units.inchesToMeters(0.5)) {
        ff = STATIC_FRICTION_CONSTANT * drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE;
      }

      final Rotation2d directionOfTravel = translationToPoint.getAngle();
      final double velocity =
          Math.min(
              Math.abs(driveToPointController.calculate(linearDistance, 0)) + ff,
              MAX_DRIVE_TO_POINT_SPEED);
      final double xSpeed = velocity * directionOfTravel.getCos();
      final double ySpeed = velocity * directionOfTravel.getSin();

      drivetrain.drive(
          xSpeed, ySpeed, targetPose.getRotation().getDegrees(), Swerve.DriveMode.DRIVE_TO_POINT);
    } else if (currentPose != null && targetPose != null) {
      final ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs();
      drivetrain.drive(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          currentPose.getRotation().getDegrees(),
          // targetPose.getRotation().getDegrees(),
          Swerve.DriveMode.DRIVE_TO_POINT);
    } else {
      final ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs();
      drivetrain.drive(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          Swerve.DriveMode.FIELD_RELATIVE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getTargetPose(
      Pose2d currentPose, RobotState robotState, CoralPositions coralPosition) {

    boolean isBlueAlliance = true;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        isBlueAlliance = false;
      }
    }

    List<CoralPositions> leftPositions =
        Arrays.asList(
            CoralPositions.LEFT_L1,
            CoralPositions.LEFT_L2,
            CoralPositions.LEFT_L3,
            CoralPositions.LEFT_L4);

    List<CoralPositions> rightPositions =
        Arrays.asList(
            CoralPositions.RIGHT_L1,
            CoralPositions.RIGHT_L2,
            CoralPositions.RIGHT_L3,
            CoralPositions.RIGHT_L4);

    Pose2d targetPose = null;
    String coralSide =
        rightPositions.contains(coralPosition)
            ? "right"
            : leftPositions.contains(coralPosition) ? "left" : "";

    List<Pose2d> redLeftScoringPoints =
        Arrays.asList(
            Constants.ScoringConstants.RED_FRONT_LEFT,
            Constants.ScoringConstants.RED_FRONT_LEFT_LEFT,
            Constants.ScoringConstants.RED_BACK_LEFT_LEFT,
            Constants.ScoringConstants.RED_BACK_LEFT,
            Constants.ScoringConstants.RED_BACK_RIGHT_LEFT,
            Constants.ScoringConstants.RED_FRONT_RIGHT_LEFT);

    List<Pose2d> redRightScoringPoints =
        Arrays.asList(
            Constants.ScoringConstants.RED_FRONT_RIGHT,
            Constants.ScoringConstants.RED_FRONT_LEFT_RIGHT,
            Constants.ScoringConstants.RED_BACK_LEFT_RIGHT,
            Constants.ScoringConstants.RED_BACK_RIGHT,
            Constants.ScoringConstants.RED_BACK_RIGHT_RIGHT,
            Constants.ScoringConstants.RED_FRONT_RIGHT_RIGHT);

    List<Pose2d> blueLeftScoringPoints =
        Arrays.asList(
            Constants.ScoringConstants.BLUE_FRONT_LEFT,
            Constants.ScoringConstants.BLUE_FRONT_LEFT_LEFT,
            Constants.ScoringConstants.BLUE_BACK_LEFT_LEFT,
            Constants.ScoringConstants.BLUE_BACK_LEFT,
            Constants.ScoringConstants.BLUE_BACK_RIGHT_LEFT,
            Constants.ScoringConstants.BLUE_FRONT_RIGHT_LEFT);

    List<Pose2d> blueRightScoringPoints =
        Arrays.asList(
            Constants.ScoringConstants.BLUE_FRONT_RIGHT,
            Constants.ScoringConstants.BLUE_FRONT_LEFT_RIGHT,
            Constants.ScoringConstants.BLUE_BACK_LEFT_RIGHT,
            Constants.ScoringConstants.BLUE_BACK_RIGHT,
            Constants.ScoringConstants.BLUE_BACK_RIGHT_RIGHT,
            Constants.ScoringConstants.BLUE_FRONT_RIGHT_RIGHT);

    List<Pose2d> scoringLocations = null;
    if (isBlueAlliance) {
      switch (coralPosition) {
        case LEFT_L1, LEFT_L2, LEFT_L3, LEFT_L4:
          scoringLocations = blueLeftScoringPoints;
          break;
        case RIGHT_L1, RIGHT_L2, RIGHT_L3, RIGHT_L4:
          scoringLocations = blueRightScoringPoints;
          break;
        default:
          return null;
      }
    } else {
      switch (coralPosition) {
        case LEFT_L1, LEFT_L2, LEFT_L3, LEFT_L4:
          scoringLocations = redLeftScoringPoints;
          break;
        case RIGHT_L1, RIGHT_L2, RIGHT_L3, RIGHT_L4:
          scoringLocations = redRightScoringPoints;
          break;
        default:
          return null;
      }
    }

    double leastDistance = 0;
    double indexOfClosestPoint;

    for (int i = 0; i < 4; i++) {
      Pose2d testPoint = scoringLocations.get(i);
      Translation2d distanceTranslation =
          currentPose.getTranslation().minus(testPoint.getTranslation());
      double distance = distanceTranslation.getNorm();

      if (i == 0 || distance < leastDistance) {
        leastDistance = distance;
        indexOfClosestPoint = i;
        targetPose = testPoint;
      }
    }

    // fill in left and right for blue from constants
    // List<Pose2d> redLeftScoringPoints = Arrays.asList()

    // pick the correct red/blue and right/left list
    // List<Pose2d> scoringLocations;

    // double leastDistance;
    // double indexOfClosestPoint;

    // for (int i = 0; i < 4; i++) {
    //   Pose2d testPoint = scoringLocations.get(i);
    //   Translation2d distanceTranslation =
    // currentPose.getTranslation().minus(testPoint.getTranslation());
    //   double distance = distanceTranslation.getNorm();

    // check for blank initial values

    //   if (i == 0 || distance < leastDistance) {
    //     leastDistance = distance;
    //     indexOfClosestPoint = i;
    //   }
    // }

    //
    // logic goes here to pick the correct zone and drive to point
    //

    return targetPose;
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    boolean isBlueAlliance = true;
    final Pose2d currentPose = drivetrain.getState().Pose;

    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        isBlueAlliance = false;
      }
    }

    final double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
    final double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
    double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.1);

    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    double xVelocity = (isBlueAlliance ? -xMagnitude * MAX_SPEED : xMagnitude * MAX_SPEED) * 0.95;
    double yVelocity = (isBlueAlliance ? -yMagnitude * MAX_SPEED : yMagnitude * MAX_SPEED) * 0.95;
    double angularVelocity = angularMagnitude * MAX_ANGULAR_RATE * 0.95;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(
            drivetrain.getState().Speeds.omegaRadiansPerSecond * SKEW_COMPENSATION);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), currentPose.getRotation()),
        currentPose.getRotation().plus(skewCompensationFactor));
  }
}
