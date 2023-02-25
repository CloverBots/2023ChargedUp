// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class TelescopeSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 10;

  private final CANSparkMax motor = new CANSparkMax(IDs.TELESCOPE_DEVICE, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = 0; // 0

  public static final double UPPER_ENDPOINT = 230; // 236

  /**
   * Constructs a new {@link TelescopeSubsystem} instance.
   */
  public TelescopeSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);

    // setTelescopeMaximumPosition(LOWER_ENDPOINT, UPPER_ENDPOINT);
  }

  public void setTelescopeSpeed(double speed, boolean byPassSafety) {

    SmartDashboard.putNumber("Telescope Encoder", getTelescopeEncoderPosition());

    // positive speed extends the telescope
    if (!byPassSafety) {
      if ((getTelescopeEncoderPosition() <= LOWER_ENDPOINT && speed < 0) ||
          (getTelescopeEncoderPosition() >= UPPER_ENDPOINT && speed > 0)) {
        speed = 0;
      }
    }
    
    motor.set(speed);
  }

  public double getTelescopeEncoderPosition() {
    return motor.getEncoder().getPosition(); // negative because motor flipped
  }

  public void setTelescopeMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

}
