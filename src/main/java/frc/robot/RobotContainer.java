package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeWristSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // Subsystems
  private final Swerve drive;
  //   private final Vision vision;
  private final RobotContainer robotContainer = this;
  private final StateMachine stateMachine;
  private final HopperSubsystem hopperSubsystem;
  private final AlgaeWristSubsystem algaeWristSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SensorsSubsystem sensorsSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final EndEffectorSubsystem endEffectorSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController characterizationController = new CommandXboxController(1);

  private final Telemetry logger = new Telemetry(MAX_SPEED);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    hopperSubsystem = new HopperSubsystem();
    algaeWristSubsystem = new AlgaeWristSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    sensorsSubsystem = new SensorsSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    endEffectorSubsystem = new EndEffectorSubsystem();
    climberSubsystem = new ClimberSubsystem();
    drive = TunerConstants.createDrivetrain();
    vision = new Vision();

    drive.resetPose(new Pose2d());

    stateMachine =
        new StateMachine(
            hopperSubsystem,
            algaeWristSubsystem,
            algaeIntakeSubsystem,
            sensorsSubsystem,
            elevatorSubsystem,
            endEffectorSubsystem,
            climberSubsystem,
            drive);

    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Real robot, instantiate hardware IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIOPigeon2(),
    //             new ModuleIOTalonFX(TunerConstantsAK.FrontLeft),
    //             new ModuleIOTalonFX(TunerConstantsAK.FrontRight),
    //             new ModuleIOTalonFX(TunerConstantsAK.BackLeft),
    //             new ModuleIOTalonFX(TunerConstantsAK.BackRight));

    //     vision =
    //         new Vision(
    //             drive::addVisionMeasurement,
    //             new VisionIOLimelight(camera0Name, drive::getRotation),
    //             new VisionIOLimelight(camera1Name, drive::getRotation));
    //     // vision =
    //     //     new Vision(
    //     //         demoDrive::addVisionMeasurement,
    //     //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
    //     //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
    //     break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIOSim(TunerConstantsAK.FrontLeft),
    //             new ModuleIOSim(TunerConstantsAK.FrontRight),
    //             new ModuleIOSim(TunerConstantsAK.BackLeft),
    //             new ModuleIOSim(TunerConstantsAK.BackRight));

    //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    //     break;

    //   default:
    //     // Replayed robot, disable IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {});

    //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    //     break;
    // }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureCharacterizationButtonBindings();
    drive.registerTelemetry(logger::telemeterize);
  }

  public void updatePose() {
    final String bestLimelight = vision.getBestLimelight();

    if (bestLimelight.isEmpty()) {
      return;
    }

    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(bestLimelight);
    LimelightHelpers.PoseEstimate bestMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight);

    if (bestMeasurement != null && bestMeasurement.avgTagArea > 0.1) {
      Pose2d newPose = pose.toPose2d();
      drive.resetRotation(newPose.getRotation());
      LimelightHelpers.SetRobotOrientation(
          "limelight-left", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(
          "limelight-right", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    } else {
      return;
    }

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
    // table.getEntry("tag-count").setDouble(mt2.tagCount);
    // table.getEntry("angular-velo").setBoolean(Math.abs(subsystems.getDrivetrain().getPigeon2().getRate()) < 720);

    if (mt2 != null
        && drive.getPigeon2().getRate() < 720
        && mt2.tagCount
            > 0) // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    {
      drive.resetPose(mt2.pose);
      drive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      drive.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
    }
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(new SwerveDriveCommand(drive, controller, stateMachine, false));

    controller.a().whileTrue(new SwerveDriveCommand(drive, controller, stateMachine, true));
    controller.a().onFalse(new SwerveDriveCommand(drive, controller, stateMachine, false));
    controller.y().whileTrue(new SwerveDriveCommand(drive, controller, stateMachine, true));
    controller.y().onFalse(new SwerveDriveCommand(drive, controller, stateMachine, false));

    controller.x().whileTrue(new SwerveDriveCommand(drive, controller, stateMachine, true));
    controller.x().onFalse(new SwerveDriveCommand(drive, controller, stateMachine, false));
    controller.b().whileTrue(new SwerveDriveCommand(drive, controller, stateMachine, true));
    controller.b().onFalse(new SwerveDriveCommand(drive, controller, stateMachine, false));

    // Lock to 0° when A button is held
    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed

    // controller
    //     .start()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    controller
        .leftTrigger(0.5)
        .onTrue(
            new InstantCommand(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.LEFT_L2)));

    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.LEFT_L4)));

    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.LEFT_L3)));

    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.RIGHT_L2)));

    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.RIGHT_L4)));

    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setSelectedCoralPosition(StateMachine.CoralPositions.RIGHT_L3)));

    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setWantedState(StateMachine.RobotState.DESCORE_ALGAE_LOWER)))
        .onFalse(
            Commands.runOnce(
                () -> stateMachine.setWantedState(StateMachine.RobotState.READY_STATE)));

    controller
        .rightBumper()
        .and(controller.leftBumper())
        .onTrue(
            Commands.runOnce(
                () -> stateMachine.setWantedState(StateMachine.RobotState.DESCORE_ALGAE_UPPER)))
        .onFalse(
            Commands.runOnce(
                () -> stateMachine.setWantedState(StateMachine.RobotState.READY_STATE)));
  }

  private void configureCharacterizationButtonBindings() {
    // Example POV button binding
    characterizationController
        .a()
        .whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    characterizationController
        .x()
        .whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    characterizationController.b().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    characterizationController.y().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // new POVButton(driverController, 0)
  // .whenPressed();

  public HopperSubsystem getHopper() {
    return hopperSubsystem;
  }

  public Swerve getDrive() {
    return drive;
  }

  public RobotContainer getRobotContainer() {
    return robotContainer;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }
}
