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
  private final DoubleSupplier trigger;
  private final DoubleSupplier leftJoystickY;
  public static final double UPPER_ENDPOINT = 87; // in rotations
  private final double LOWER_ENDPOINT = 0.0;
  private final double APPROACH_MAX_SPEED = 0.2;

  /** Creates a new LiftCommand. */
  public WristCommand(WristSubsystem wristSubsystem, DoubleSupplier trigger, DoubleSupplier leftJoystickY) {
    this.wristSubsystem = wristSubsystem;
    this.trigger = trigger;
    this.leftJoystickY = leftJoystickY;

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

    if (trigger.getAsDouble() > .5) {
      double wristSpeed = leftJoystickY.getAsDouble() * .5;

      if ((wristSubsystem.getWristEncoderPosition() <= LOWER_ENDPOINT && wristSpeed > 0) ||
          (wristSubsystem.getWristEncoderPosition() >= UPPER_ENDPOINT && wristSpeed < 0)) {
        wristSpeed = 0;
      }

      if (wristSubsystem.getWristEncoderPosition() - LOWER_ENDPOINT < 3
          || UPPER_ENDPOINT - wristSubsystem.getWristEncoderPosition() < 3) {
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