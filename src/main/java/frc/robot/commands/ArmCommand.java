// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final DoubleSupplier leftJoystickY;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem armSubsystem, DoubleSupplier leftJoystickY) {
    this.armSubsystem = armSubsystem;
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

    double armSpeed = -leftJoystickY.getAsDouble() * .5; //negative because joystick Y is negative for forward push of joystick

    if (Math.abs(armSpeed) < 0.05) { //dead-zone to prevent controller drift
      armSpeed = 0;
    }
    
    armSubsystem.setArmSpeed(armSpeed);
    // if (Math.abs(armSpeed) > 0.05) {
    
    //   if (armSubsystem.getArmEncoderPosition() - ArmSubsystem.LOWER_ENDPOINT < APPROACH_ENCODER_LIMIT
    //       || ArmSubsystem.UPPER_ENDPOINT - armSubsystem.getArmEncoderPosition() < APPROACH_ENCODER_LIMIT) {
    //     armSpeed = Math.min(Math.max(armSpeed, -APPROACH_MAX_SPEED), APPROACH_MAX_SPEED);
    //   }
      
    //     armSubsystem.setArmSpeed(armSpeed);
    // } else armSubsystem.setArmSpeed(0);
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