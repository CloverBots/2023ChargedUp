// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SpinToAngleCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private double speed;
  private Timer timer;

  
  /** Creates a new SpinToAngleCommand. */
  public SpinToAngleCommand(DriveSubsystem driveSubsystem, double rotation, double speed) {
    this.driveSubsystem = driveSubsystem;
    this.rotation = rotation;
    this.speed = speed;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveSubsystem.navXGyro.reset();
    driveSubsystem.driveRotatePidController.setSetpoint(rotation);
    driveSubsystem.driveDistancePidController.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveRotatePidController.setP(SmartDashboard.getNumber("drive rotate kP", DriveSubsystem.DRIVEROTATE_PID_P));
    driveSubsystem.driveRotatePidController.setD(SmartDashboard.getNumber("drive rotate kD", DriveSubsystem.DRIVEROTATE_PID_D));
    double calc = -driveSubsystem.driveRotatePidController.calculate(driveSubsystem.navXGyro.getHeading());
    SmartDashboard.putNumber("drive rotate PID calculation", calc);
    driveSubsystem.autoDrive(0, calc);
    //driveSubsystem.arcadeDrive(0, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0,0);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(rotation - driveSubsystem.navXGyro.getHeading()) < driveSubsystem.driveRotatePidController.getPositionTolerance() || timer.hasElapsed(1);
    // if (rotation > 0 && driveSubsystem.navXGyro.getHeading() > rotation) {
    //   return true;
    // } else if (rotation < 0 && driveSubsystem.navXGyro.getHeading() < rotation) {
    //   return true;
    // } else {
    //   return false;
    // }
    //return driveSubsystem.driveRotatePidController.atSetpoint();
  }
}
