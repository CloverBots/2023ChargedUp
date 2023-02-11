// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class IntakeToPositionCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final TelescopeSubsystem telescopeSubsystem;
  private final WristSubsystem wristSubsystem;

  private double armPosition;
  private double armSpeed;
  private double telescopePosition;
  private double telescopeSpeed;
  private double wristPosition;
  private double wristSpeed;

  private int armDirection = 1;
  private int telescopeDirection = 1;
  private int wristDirection = 1;

  /** Creates a new IntakeToPosition. */
  public IntakeToPositionCommand(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem, 
    WristSubsystem wristSubsystem, double armPosition, double armSpeed, 
    double telescopePosition, double telescopeSpeed, double wristPosition, double wristSpeed) {
    
    
    if (armPosition > ArmSubsystem.UPPER_ENDPOINT) {
      armPosition = ArmSubsystem.UPPER_ENDPOINT;
    }

    if (armPosition < ArmSubsystem.LOWER_ENDPOINT) {
      armPosition = ArmSubsystem.LOWER_ENDPOINT;
    }

    if (telescopePosition > TelescopeSubsystem.UPPER_ENDPOINT) {
      telescopePosition = TelescopeSubsystem.UPPER_ENDPOINT;
    }

    if (telescopePosition < TelescopeCommand.LOWER_ENDPOINT) {
      telescopePosition = TelescopeCommand.LOWER_ENDPOINT;
    }

    if (wristPosition > WristSubsystem.UPPER_ENDPOINT) {
      wristPosition = WristSubsystem.UPPER_ENDPOINT;
    }

    if (wristPosition < WristSubsystem.LOWER_ENDPOINT) {
      wristPosition = WristSubsystem.LOWER_ENDPOINT;
    }

    this.armSubsystem = armSubsystem;
    this.telescopeSubsystem = telescopeSubsystem;
    this.wristSubsystem = wristSubsystem;

    this.armPosition = armPosition;
    this.armSpeed = armSpeed;
    this.telescopePosition = telescopePosition;
    this.telescopeSpeed = telescopeSpeed;
    this.wristPosition = wristPosition;
    this.wristSpeed = wristSpeed;

    addRequirements(armSubsystem);
    addRequirements(wristSubsystem);
    addRequirements(telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
armSubsystem.resetEncoder();
wristSubsystem.resetEncoder();


    if (armSubsystem.getArmEncoderPosition() > armPosition) {
      armDirection = -1; // going down
    }

    if (telescopeSubsystem.getTelescopeEncoderPosition() > telescopePosition) {
      telescopeDirection = -1; // going down
    }

    if (wristSubsystem.getWristEncoderPosition() > wristPosition) {
      wristDirection = -1; // going down
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmSpeed(armSpeed * armDirection);
    //telescopeSubsystem.setTelescopeSpeed(telescopeSpeed * telescopeDirection);
    wristSubsystem.setWristSpeed(wristSpeed * wristDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmSpeed(0);
    //telescopeSubsystem.setTelescopeSpeed(0);
    wristSubsystem.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean armDone = false;
    boolean telescopeDone = true;
    boolean wristDone = false;

    SmartDashboard.putNumber("Arm position: ", armSubsystem.getArmEncoderPosition());
    SmartDashboard.putNumber("Wrist position: ", wristSubsystem.getWristEncoderPosition());
    if (armDirection == -1 && armSubsystem.getArmEncoderPosition() <= armPosition) {
      armDone = true;
      armSubsystem.setArmSpeed(0);
    } else if (armDirection == 1 && armSubsystem.getArmEncoderPosition() >= armPosition) {
      armDone = true;
      armSubsystem.setArmSpeed(0);
    }
/**
    if (telescopeDirection == -1 && telescopeSubsystem.getEncoderPosition() <= telescopePosition) {
      telescopeDone = true;
      telescopeSubsystem.setTelescopeSpeed(0);
    } else if (telescopeDirection == 1 && telescopeSubsystem.getEncoderPosition() >= telescopePosition) {
      telescopeDone = true;
      telescopeSubsystem.setTelescopeSpeed(0);
    }
*/
    if (wristDirection == -1 && wristSubsystem.getWristEncoderPosition() <= wristPosition) {
      wristDone = true;
      wristSubsystem.setWristSpeed(0);
    } else if (wristDirection == 1 && wristSubsystem.getWristEncoderPosition() >= wristPosition) {
      wristDone = true;
      wristSubsystem.setWristSpeed(0);
    }

    return armDone && wristDone && telescopeDone;

  }
}
