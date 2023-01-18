package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem2;

public class TriMotorTestCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    private LiftSubsystem2 liftSubsystem2;
    
    private static final double MOTOR_0_ENDPOINT = -40;
    private static final double MOTOR_1_ENDPOINT = -20;

    private boolean motor1Run = true;
    private boolean motor2Run = true;

    private double initialEncoder1 = -1;
    private double initialEncoder2 = -1;



    public TriMotorTestCommand(LiftSubsystem liftSubsystem, LiftSubsystem2 liftSubsystem2) {
        this.liftSubsystem = liftSubsystem;
        this.liftSubsystem2 = liftSubsystem2;
    }

    @Override
    public void initialize() {
        System.out.println("Initialize xxxxxxxxxxxxxxxxxxx");
        initialEncoder1 = liftSubsystem.getLiftEncoderPosition();
        initialEncoder2 = liftSubsystem2.getLiftEncoderPosition();
        motor1Run = true;
        motor2Run = true;
    }

    @Override
    public void execute() {
        if (motor1Run) {
            liftSubsystem.setLiftSpeed(0.1);
        }
        if (motor2Run) {
            liftSubsystem2.setLiftSpeed(0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        System.out.println("lift 0: "+liftSubsystem.getLiftEncoderPosition());
        System.out.println("lift 1: "+liftSubsystem2.getLiftEncoderPosition());

        if (liftSubsystem.getLiftEncoderPosition() <= MOTOR_0_ENDPOINT + initialEncoder1) {
            liftSubsystem.setLiftSpeed(0);
            motor1Run = false;
        }

        if (liftSubsystem2.getLiftEncoderPosition() <= MOTOR_1_ENDPOINT + initialEncoder2) {
            liftSubsystem2.setLiftSpeed(0);
            motor2Run = false;
        } 

        return !motor1Run && !motor2Run;
    }

    
    
}
