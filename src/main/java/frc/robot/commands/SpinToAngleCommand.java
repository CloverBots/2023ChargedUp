// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SpinToAngleCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private double speed;
  
  /** Creates a new SpinToAngleCommand. */
  public SpinToAngleCommand(DriveSubsystem driveSubsystem, double rotation, double speed) {
    this.driveSubsystem = driveSubsystem;
    this.rotation = rotation;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.navXGyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(0, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rotation > 0 && driveSubsystem.navXGyro.getHeading() > rotation) {
      return true;
    } else if (rotation < 0 && driveSubsystem.navXGyro.getHeading() < rotation) {
      return true;
    } else {
      return false;
    }
  }
}
