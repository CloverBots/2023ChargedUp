// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NavXGyro;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToBalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double distance;
    private final double speed;
    private NavXGyro gyro;
    private double currentRoll;
    private double initialRoll;

    /**
     * Creates a new DriveToDistance.
     * 
     * @param driveSubsystem what subsystem to use
     * @param distance       distance driven forward in meters
     * @param maxSpeed          set the maximum speed
     * @param rotate         how much to roatate in degrees
     */

    public DriveToBalanceCommand(DriveSubsystem driveSubsystem, double distance, double speed) {
        this.distance = distance;
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
        addRequirements(driveSubsystem);
        gyro = driveSubsystem.navXGyro;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveSubsystem.resetEncoders();
        initialRoll = gyro.getRoll();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distanceTraveled = driveSubsystem.getAverageEncoderPosition();
        SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
        driveSubsystem.autoDrive(speed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the robot from driving forward when the robot has reached the target
        // position
        driveSubsystem.autoDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        currentRoll = gyro.getRoll();
        double distanceTraveled = driveSubsystem.getAverageEncoderPosition();
        if (Math.abs(distanceTraveled) >= Math.abs(distance)) {
            System.out.println("Finishing due to distance");
            return true;
        } else if (Math.abs(currentRoll - initialRoll) > 5){
            System.out.println("Finishing due to roll");
            return true;
        } else {
            return false;
        }
    }
}
