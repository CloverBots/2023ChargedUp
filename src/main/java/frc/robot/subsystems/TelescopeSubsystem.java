// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;


public class TelescopeSubsystem extends SubsystemBase {

  public static final double WHEEL_DIAMETER_METERS = 0.1524;
  public static final double ENCODER_POSITION_CONVERSION_FACTOR = 0.1 * WHEEL_DIAMETER_METERS * Math.PI;
  public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = ENCODER_POSITION_CONVERSION_FACTOR * 60.0;
  public static final double ENCODER_TICKS_PER_ROTATION = 2048;

  private final TalonFX motor = new TalonFX(IDs.TELESCOPE_DEVICE);

  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem() {
    motor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getEncoderPosition() {
    return motor.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION * ENCODER_POSITION_CONVERSION_FACTOR;
  }

  public void resetEncoders() {
    motor.setSelectedSensorPosition(0);

  }
}
