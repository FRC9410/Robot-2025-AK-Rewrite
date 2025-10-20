package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
  private final TalonFX endEffectorMotor;
  private static final NeutralOut brake = new NeutralOut();
  private double voltage;

  private final MotionMagicVelocityTorqueCurrentFOC motionMagicRequest;

  /** Constructor for the EndEffector subsystem. */
  public EndEffectorSubsystem() {
    endEffectorMotor =
        new TalonFX(Constants.EndEffectorConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

    endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);

    endEffectorMotor.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);

    motionMagicRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs
        // .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.MOTION_MAGIC_VELOCITY)
        .withMotionMagicAcceleration(Constants.ElevatorConstants.MOTION_MAGIC_ACCELERATION);

    // Configure PID
    config.Slot0.kP = 40.0;

    endEffectorMotor.getConfigurator().apply(config);
    endEffectorMotor.getConfigurator().apply(motionMagicConfigs);
    endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  public void intakeCoral() {
    setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE);
  }

  public void outtakeCoral() {
    setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE);
  }

  public void stopMotors() {
    setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);
  }

  /**
   * Runs the hopper motors at the given speed.
   *
   * <p>The primary motor is commanded to run at the specified speed while the secondary motor runs
   * in reverse (i.e. with the negative of the speed).
   *
   * @param speed The desired motor output (range between -1 and 1).
   */
  public void setVoltage(double voltage) {
    if (voltage != this.voltage) {
      this.voltage = voltage;
      endEffectorMotor.setVoltage(voltage);
    }
    // No need to command secondaryMotor here since it follows primaryMotor in reverse.
  }

  public void setVelocity(double velocity) {
    if (velocity == 0) {
      endEffectorMotor.setControl(brake);
    } else {
      endEffectorMotor.setControl(motionMagicRequest.withVelocity(velocity).withSlot(0));
    }
  }

  public boolean isReady() {
    if (voltage == Constants.EndEffectorConstants.STOP_VOLTAGE) {
      return true;
    } else {
      return false;
    }
  }
}
