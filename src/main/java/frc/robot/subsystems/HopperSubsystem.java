// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;
  private static final NeutralOut brake = new NeutralOut();
  //private final BiConsumer<String, Object> updateData;
  private double voltage;

  /**
   * Hopper subsystem constructor.
   *
   * @param primaryMotorID CAN ID for the primary motor.
   * @param secondaryMotorID CAN ID for the secondary motor.
   */
  public HopperSubsystem() {
    primaryMotor =
        new TalonFX(
            Constants.HopperConstants.PRIMARY_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
    secondaryMotor =
        new TalonFX(
            Constants.HopperConstants.SECONDARY_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

    // Configure neutral mode to brake on both motors
    primaryMotor.setNeutralMode(NeutralModeValue.Coast);
    secondaryMotor.setNeutralMode(NeutralModeValue.Coast);

    // Optionally, configure additional settings (if needed)
    TalonFXConfiguration config = new TalonFXConfiguration();
    primaryMotor.getConfigurator().apply(config);
    secondaryMotor.getConfigurator().apply(config);
    secondaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));

   //this.updateData = updateData;
    primaryMotor.setVoltage(Constants.HopperConstants.STOP_VOLTAGE);
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
      primaryMotor.setVoltage(voltage);
    }
    // No need to command secondaryMotor here since it follows primaryMotor in reverse.
  }

  @Override
  public void periodic() {
   // updateData.accept(Constants.MapConstants.HOPPER_VOLTAGE, voltage);
    // Post the current state to network tables or smth

  }
}
