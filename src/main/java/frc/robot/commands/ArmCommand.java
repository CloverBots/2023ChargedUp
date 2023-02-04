// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final DoubleSupplier trigger;
  private final DoubleSupplier leftJoystickY;
  public static final double UPPER_ENDPOINT = 87; // in rotations
  private final double LOWER_ENDPOINT = 0.0;
  private final double APPROACH_MAX_SPEED = 0.2;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem armSubsystem, DoubleSupplier trigger, DoubleSupplier leftJoystickY) {
    this.armSubsystem = armSubsystem;
    this.trigger = trigger;
    this.leftJoystickY = leftJoystickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("elevator height", armSubsystem.getArmEncoderPosition());

    if (trigger.getAsDouble() > .5) {
      double armSpeed = leftJoystickY.getAsDouble() * .5;

      if ((armSubsystem.getArmEncoderPosition() <= LOWER_ENDPOINT && armSpeed > 0) ||
          (armSubsystem.getArmEncoderPosition() >= UPPER_ENDPOINT && armSpeed < 0)) {
        armSpeed = 0;
      }

      if (armSubsystem.getArmEncoderPosition() - LOWER_ENDPOINT < 3
          || UPPER_ENDPOINT - armSubsystem.getArmEncoderPosition() < 3) {
        armSpeed = Math.min(Math.max(armSpeed, -APPROACH_MAX_SPEED), APPROACH_MAX_SPEED);
      }
      
        armSubsystem.setArmSpeed(armSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}