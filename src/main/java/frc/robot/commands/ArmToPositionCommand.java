// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double ARM_SPEED = 0.4;
  private int position;
  private int direction;

  /** Creates a new LiftCommand. */
  public ArmToPositionCommand(ArmSubsystem armSubsystem, int position) {
    this.armSubsystem = armSubsystem;

    //Guard against too large of a position value
    if (position > ArmSubsystem.UPPER_ENDPOINT) {
      position = (int) ArmSubsystem.UPPER_ENDPOINT;
    }
    
    this.position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.resetEncoder();
    direction = 1; // going up
    if (armSubsystem.getArmEncoderPosition() > position) {
      direction = -1; // going down
    }
    SmartDashboard.putNumber("Arm starting position: ", armSubsystem.getArmEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm position: ", armSubsystem.getArmEncoderPosition());
    armSubsystem.setArmSpeed(ARM_SPEED * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction == -1 && armSubsystem.getArmEncoderPosition() <= position) {
      return true;
    } else if (direction == 1 && armSubsystem.getArmEncoderPosition() >= position) {
      return true;
    } else {
      return false;
    }
  }
}