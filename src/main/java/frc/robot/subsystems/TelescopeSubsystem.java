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

  private final CANSparkMax leadMotor = new CANSparkMax(IDs.TELESCOPE_DEVICE_LEAD, MotorType.kBrushless);
  private final CANSparkMax followMotor = new CANSparkMax(IDs.TELESCOPE_DEVICE_FOLLOW, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = 2; // 0, (slightly above 0 to prevent overshoot)

  public static final double UPPER_ENDPOINT = 230; // 236

  /**
   * Constructs a new {@link TelescopeSubsystem} instance.
   */
  public TelescopeSubsystem() {
    leadMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    leadMotor.setIdleMode(IdleMode.kBrake);

    leadMotor.setInverted(false);

    followMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    followMotor.setIdleMode(IdleMode.kBrake);

    followMotor.setInverted(false);

    followMotor.follow(leadMotor);

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
    
    leadMotor.set(speed);
  }

  public double getTelescopeEncoderPosition() {
    return leadMotor.getEncoder().getPosition(); // negative because motor flipped
  }

  public void setTelescopeMaximumPosition(double min, double max) {
    leadMotor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    leadMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
  }

  public void resetEncoder() {
    leadMotor.getEncoder().setPosition(0);
  }

}
