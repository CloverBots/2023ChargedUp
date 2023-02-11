// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  private final WristSubsystem wristSubsystem;
  private final DoubleSupplier rightJoystickY;
  public static final double LOWER_ENDPOINT = 0.0;
  private final double APPROACH_MAX_SPEED = 0.2;

  /** Creates a new LiftCommand. */
  public WristCommand(WristSubsystem wristSubsystem, DoubleSupplier rightJoystickY) {
    this.wristSubsystem = wristSubsystem;
    this.rightJoystickY = rightJoystickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Wrist", wristSubsystem.getWristEncoderPosition());

    double wristSpeed = rightJoystickY.getAsDouble() * .5;
    if (Math.abs(wristSpeed) > 0.05) {
      
      if ((wristSubsystem.getWristEncoderPosition() <= LOWER_ENDPOINT && wristSpeed > 0) ||
          (wristSubsystem.getWristEncoderPosition() >= WristSubsystem.UPPER_ENDPOINT && wristSpeed < 0)) {
        wristSpeed = 0;
      }

      if (wristSubsystem.getWristEncoderPosition() - LOWER_ENDPOINT < 3
          || WristSubsystem.UPPER_ENDPOINT - wristSubsystem.getWristEncoderPosition() < 3) {
        wristSpeed = Math.min(Math.max(wristSpeed, -APPROACH_MAX_SPEED), APPROACH_MAX_SPEED);
      }
      
        wristSubsystem.setWristSpeed(wristSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}