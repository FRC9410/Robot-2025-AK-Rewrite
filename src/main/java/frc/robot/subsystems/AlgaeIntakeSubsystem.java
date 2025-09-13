package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BiConsumer;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private static final NeutralOut brake = new NeutralOut();
  private final BiConsumer<String, Object> updateData;
  private double voltage;
  private boolean hasGamePiece = false;
  private static final VelocityVoltage voltageVelocity = new VelocityVoltage(-100);

  /**
   * Constructor for the Algae Intake subsystem.
   *
   * @param motorID CAN ID for the intake motor.
   */
  public AlgaeIntakeSubsystem(BiConsumer<String, Object> updateData) {
    intakeMotor =
        new TalonFX(Constants.AlgaeIntakeConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

    // Configure motor controller using the configTalonFx method.
    configTalonFx();

    // Set neutral mode; coast mode is often preferred for an intake.
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    this.updateData = updateData;

    voltage = Constants.AlgaeIntakeConstants.STOP_VOLTAGE;
    setIntakeConfigs(intakeMotor);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Runs the intake motor at the specified speed.
   *
   * <p>Positive speeds might be used for intake, while negative speeds reverse the motor for
   * outtake. Adjust your usage per your mechanism's design.
   *
   * @param speed The motor output in the range [-1, 1].
   */
  public void setVoltage(double speed) {
    intakeMotor.setControl(new DutyCycleOut(speed));
    // if (voltage != this.voltage) {
    //     this.voltage = voltage;
    //     intakeMotor.setVoltage(voltage);
    // }
  }

  @Override
  public void periodic() {
    // System.out.println(getHasGamePiece());
  }

  /**
   * Holds (stalls) the intake by commanding the motor to maintain its current position using
   * closed-loop PID control.
   */
  public void stallIntake() {
    // this.intakeMotor.setControl(voltageVelocity.withVelocity(25).withFeedForward(-9));
    intakeMotor.setControl(new DutyCycleOut(0.2));
  }

  public void intakeAlgae() {
    this.intakeMotor.setControl(voltageVelocity.withVelocity(50).withFeedForward(10));
  }

  public void outtakeAlgae() {
    this.intakeMotor.setControl(voltageVelocity.withVelocity(-50).withFeedForward(-8));
    setHasGamePiece(false);
  }

  public void stopIntake() {
    this.intakeMotor.setControl(brake);
  }

  public double getVelocity() {
    return intakeMotor.getRotorVelocity().getValueAsDouble();
  }

  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }

  public boolean getHasGamePiece() {
    return hasGamePiece;
  }

  /**
   * Configures the TalonFX motor controller with default settings. Adjust any motor-specific tuning
   * parameters as needed.
   */
  private void configTalonFx() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Set slot 0 PID constants using values from Constants
    config.Slot0.kP = Constants.AlgaeIntakeConstants.kP;
    config.Slot0.kI = Constants.AlgaeIntakeConstants.kI;
    config.Slot0.kD = Constants.AlgaeIntakeConstants.kD;
    config.Slot0.kG = Constants.AlgaeIntakeConstants.kF;
    // Apply the configuration
    intakeMotor.getConfigurator().apply(config);
  }

  private static void setIntakeConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.3; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(configs);
  }
}
