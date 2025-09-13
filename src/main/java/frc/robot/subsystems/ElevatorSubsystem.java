package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BiConsumer;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor =
      new TalonFX(
          Constants.ElevatorConstants.PRIMARY_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
  private final TalonFX elevatorFollowerMotor =
      new TalonFX(
          Constants.ElevatorConstants.SECONDARY_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
  private final MotionMagicVoltage motionMagicRequest;
  private final BiConsumer<String, Object> updateData;
  private double voltage;
  private double setpoint;

  public ElevatorSubsystem(BiConsumer<String, Object> updateData) {
    // Create the position request for closed-loop control
    motionMagicRequest = new MotionMagicVoltage(0);

    // Configure the Kraken
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs
        .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.MOTION_MAGIC_VELOCITY)
        .withMotionMagicAcceleration(Constants.ElevatorConstants.MOTION_MAGIC_ACCELERATION);

    // Configure PID
    config.Slot0.kP = Constants.ElevatorConstants.kP;
    config.Slot0.kI = Constants.ElevatorConstants.kI;
    config.Slot0.kD = Constants.ElevatorConstants.kD;
    config.Slot0.kG = Constants.ElevatorConstants.kG;

    // Configure soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ElevatorConstants.L4_SCORE_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ElevatorConstants.HOME_POSITION;

    // Apply configuration
    elevatorMotor.getConfigurator().apply(config);
    elevatorMotor.getConfigurator().apply(motionMagicConfigs);
    elevatorFollowerMotor.getConfigurator().apply(followerConfig);
    elevatorMotor.setPosition(0);

    // Set brake mode
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), true));

    this.updateData = updateData;

    voltage = Constants.ElevatorConstants.STOP_VOLTAGE;
    setpoint = Constants.ElevatorConstants.HOME_POSITION;
    // elevatorMotor.setControl(motionMagicRequest.withPosition(0.25).withSlot(0));
  }

  @Override
  public void periodic() {
    updateData.accept("Elevator Position", getCurrentHeight());
    updateData.accept("Elevator Setpoint", setpoint);
  }

  /**
   * Sets the elevator to a specific height
   *
   * @param heightMeters The target height in meters
   */
  public void setPosition(double position) {
    if (position != setpoint) {
      setpoint = position;
      elevatorMotor.setControl(motionMagicRequest.withPosition(position).withSlot(0));
    }
  }

  /**
   * Gets the current height of the elevator
   *
   * @return Current height in meters
   */
  public double getCurrentHeight() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  /** Returns whether the elevator is at its target position */
  public boolean atTargetPosition() {
    return Math.abs(elevatorMotor.getPosition().getValueAsDouble() - setpoint)
        < 0.05; // 2cm tolerance
  }

  public boolean isAtPosition(double position) {
    return Math.abs(elevatorMotor.getPosition().getValueAsDouble() - position)
        < 0.05; // 2cm tolerance
  }

  public void setVoltage(double voltage) {
    if (voltage != this.voltage) {
      this.voltage = voltage;
      elevatorMotor.setVoltage(voltage);
    }
  }

  /**
   * Zeros the elevator encoder at the current position Use this when the elevator is at a known
   * position
   */
  public void zeroEncoder() {
    elevatorMotor.setPosition(0);
  }
}
