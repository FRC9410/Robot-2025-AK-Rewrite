package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BiConsumer;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final PositionVoltage positionRequest;
  private final BiConsumer<String, Object> updateData;
  private double voltage;
  private double setpoint;

  /**
   * Constructor for the Climber subsystem.
   *
   * @param motorID CAN ID for the climber's motor.
   */
  public ClimberSubsystem(BiConsumer<String, Object> updateData) {
    climberMotor =
        new TalonFX(Constants.ClimberConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
    positionRequest = new PositionVoltage(0).withSlot(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    // config.Slot0.kP = Constants.ClimberConstants.kP;
    // config.Slot0.kI = Constants.ClimberConstants.kI;
    // config.Slot0.kD = Constants.ClimberConstants.kD;
    // config.Slot0.kG = Constants.ClimberConstants.kF;

    // Set up soft limits to prevent over-extension or over-retraction.
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // Constants.ClimberConstants.WINCH_CLIMB_POSITION;
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    climberMotor.getConfigurator().apply(config);

    // Set brake mode so the winch holds its position when no power is applied.
    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    this.updateData = updateData;

    voltage = Constants.ClimberConstants.STOP_VOLTAGE;
    setpoint = 0.0;
  }

  @Override
  public void periodic() {
    // System.out.println("Climber position: " + climberMotor.getPosition().getValueAsDouble());
  }

  /**
   * Commands the climber to extend or retract to a specified position (in meters).
   *
   * @param extensionMeters The desired extension in meters.
   */
  public void setPosition(double position) {
    if (position != setpoint) {
      setpoint = position;
      climberMotor.setControl(positionRequest.withPosition(position));
    }
  }

  /**
   * Returns the current climber extension (in meters) based on the motor encoder.
   *
   * @return Current extension in meters.
   */
  public double getCurrentExtension() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    // if (voltage != this.voltage) {
    // this.voltage = voltage;
    // climberMotor.setVoltage(voltage);
    this.voltage = voltage;
    climberMotor.setControl(new DutyCycleOut(-voltage));
    // }
  }

  /** Zeroes the climber encoder. Call this when the climber is at a known reference position. */
  public void zeroEncoder() {
    climberMotor.setPosition(0);
  }

  public void stop() {
    setVoltage(Constants.ClimberConstants.STOP_VOLTAGE);
  }

  public void start() {
    setVoltage(Constants.ClimberConstants.CLIMB_VOLTAGE);
  }

  public boolean isReady() {
    if (voltage == Constants.ClimberConstants.STOP_VOLTAGE) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRunning() {
    if (voltage != Constants.ClimberConstants.STOP_VOLTAGE) {
      return true;
    } else {
      return false;
    }
  }
}
