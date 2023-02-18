// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class TelescopeSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 10;

  private final CANSparkMax motor = new CANSparkMax(IDs.TELESCOPE_DEVICE, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = -300;

  public static final double UPPER_ENDPOINT = 300; // in rotations

  /**
   * Constructs a new {@link TelescopeSubsystem} instance.
   */
  public TelescopeSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);
  }

  public void setTelescopeSpeed(double speed) {
    motor.set(speed);
  }

  public double getTelescopeEncoderPosition() {
    return -motor.getEncoder().getPosition(); // negative because goofy encoder
  }

  public void setTelescopeMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

}
