package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem2;

public class TriMotorTestCommand extends CommandBase {
    
    // private LiftSubsystem liftSubsystem;
    // private LiftSubsystem2 liftSubsystem2;
    
    private MotorRotationInfo[] motorsToRotate;

    // private static final double MOTOR_0_ENDPOINT = -40;
    // private static final double MOTOR_1_ENDPOINT = -20;

    // private boolean motor1Run = true;
    // private boolean motor2Run = true;

    // private double initialEncoder1 = -1;
    // private double initialEncoder2 = -1;


    private static final double KP = 0.002;
    private static final double KI = 0;
    private static final double KD = 0;
    
    public TriMotorTestCommand(LiftSubsystem liftSubsystem, LiftSubsystem2 liftSubsystem2, double seconds, double ... motorRotationValues) {
        motorsToRotate = new MotorRotationInfo[motorRotationValues.length];
        
        motorsToRotate[0] = new MotorRotationInfo(liftSubsystem.getMotor());
        motorsToRotate[1] = new MotorRotationInfo(liftSubsystem2.getMotor());
        for (int i=0; i<motorsToRotate.length; i++) {
            motorsToRotate[i].setRotation(motorRotationValues[i], seconds);
        }

    }

    @Override
    public void initialize() {
        System.out.println("Initialize xxxxxxxxxxxxxxxxxxx");
        // initialEncoder1 = liftSubsystem.getLiftEncoderPosition();
        // initialEncoder2 = liftSubsystem2.getLiftEncoderPosition();
        // motor1Run = true;
        // motor2Run = true;
    }

    @Override
    public void execute() {

        for (MotorRotationInfo info : motorsToRotate) {
            double calculated = info.RPMPidController.calculate(info.motor.getEncoder().getVelocity());
            System.out.println(calculated);
            info.motor.set(info.motor.get() + calculated);
        }

        // if (motor1Run) {
        //     liftSubsystem.setLiftSpeed(0.1);
        // }
        // if (motor2Run) {
        //     liftSubsystem2.setLiftSpeed(0.2);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        // System.out.println("lift 0: "+liftSubsystem.getLiftEncoderPosition());
        // System.out.println("lift 1: "+liftSubsystem2.getLiftEncoderPosition());
        
        for (int i=0; i<motorsToRotate.length; i++) {
            System.out.printf("Motor %d: %s\n", i, motorsToRotate[i].toString());
        }
        for (MotorRotationInfo info : motorsToRotate) {
            double pos = info.motor.getEncoder().getPosition();
            if (pos <= info.tickEndpoint + info.initialEncoderVal) {
                info.motor.set(0);
                info.isRunning = false;
            }
        }

        // Continues the command if any of the motors are running.
        for (MotorRotationInfo info : motorsToRotate) if (info.isRunning) return false;

        return true;

        // if (liftSubsystem.getLiftEncoderPosition() <= MOTOR_0_ENDPOINT + initialEncoder1) {
        //     liftSubsystem.setLiftSpeed(0);
        //     motor1Run = false;
        // }

        // if (liftSubsystem2.getLiftEncoderPosition() <= MOTOR_1_ENDPOINT + initialEncoder2) {
        //     liftSubsystem2.setLiftSpeed(0);
        //     motor2Run = false;
        // } 

        // return !motor1Run && !motor2Run;
    }

    private static class MotorRotationInfo {
        public final CANSparkMax motor;
        @SuppressWarnings("unused") double degrees;
        double tickEndpoint;
        boolean isRunning = true;
        double secondsToRun;
        double initialEncoderVal;
        double reqRPM;
        PIDController RPMPidController = new PIDController(KP, KI, KD);

        public MotorRotationInfo(CANSparkMax motor) {
            this.motor = motor;
        }

        public MotorRotationInfo setRotation(double degrees, double seconds) {
            this.degrees = degrees;
            secondsToRun = seconds;
            tickEndpoint = (DriveSubsystem.ENCODER_TICKS_PER_ROTATION * degrees / 360);
            initialEncoderVal = motor.getEncoder().getPosition();
            reqRPM = (degrees/360) * 60 / secondsToRun;
            RPMPidController.setSetpoint(reqRPM);
            return this;
        }

        @Override
        public String toString() {
            return String.format("Degrees: %f, Endpoint: %f, initialEncode: %f, RPM: %f", degrees, tickEndpoint, initialEncoderVal, reqRPM);
        }
    }
    
}
