package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BiConsumer;

public class EndEffectorSubsystem extends SubsystemBase {
  private final TalonFX endEffectorMotor;
  private static final NeutralOut brake = new NeutralOut();
  private final BiConsumer<String, Object> updateData;
  private double voltage;

  /** Constructor for the EndEffector subsystem. */
  public EndEffectorSubsystem(BiConsumer<String, Object> updateData) {
    endEffectorMotor =
        new TalonFX(Constants.EndEffectorConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

    endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);

    this.updateData = updateData;

    endEffectorMotor.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);
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
}
