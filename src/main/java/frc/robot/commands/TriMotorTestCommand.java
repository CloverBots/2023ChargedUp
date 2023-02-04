package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TriMotorTestCommand extends CommandBase {
 /**   
    // private LiftSubsystem liftSubsystem;
    // private LiftSubsystem2 liftSubsystem2;
    
    private MotorRotationInfo[] motorsToRotate;

    // private static final double MOTOR_0_ENDPOINT = -40;
    // private static final double MOTOR_1_ENDPOINT = -20;

    // private boolean motor1Run = true;
    // private boolean motor2Run = true;

    // private double initialEncoder1 = -1;
    // private double initialEncoder2 = -1;

    private static final double POWER_TO_RPM_CONSTANT = 5898.98920859685;

    private static final double KP = 0.002;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final Timer timer = new Timer();
    public TriMotorTestCommand(LiftSubsystem liftSubsystem, WristSubsystem liftSubsystem2, double seconds, double ... motorRotationValues) {
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
        for (MotorRotationInfo info : motorsToRotate) {
            System.out.println("factor "+info.motor.getEncoder().getVelocityConversionFactor());
            info.motor.set(0.1);
            System.out.println(info.reqRPM/POWER_TO_RPM_CONSTANT);
        }
        timer.reset();
        timer.start();
        // initialEncoder1 = liftSubsystem.getLiftEncoderPosition();
        // initialEncoder2 = liftSubsystem2.getLiftEncoderPosition();
        // motor1Run = true;
        // motor2Run = true;
    }

    @Override
    public void execute() {

        for (MotorRotationInfo info : motorsToRotate) {
            info.motor.set(info.reqRPM/POWER_TO_RPM_CONSTANT);
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
        for (MotorRotationInfo info : motorsToRotate) {
            info.motor.set(0);
        }
    }

    private double proportionalTest() {
        double[] velocities = new double[10];
        int n = 0;
        motorsToRotate[0].motor.getEncoder().setPosition(0);
        while (n<10) {
            MotorRotationInfo info = motorsToRotate[0];
            double tRound = (Math.floor(timer.get()*10))/10.0;
            if ( tRound % 1 == 0 && (tRound-1 == n)) {
                double v = info.motor.getEncoder().getPosition() / DriveSubsystem.ENCODER_TICKS_PER_ROTATION;
                velocities[n] = v;
                System.out.println("Power: "+info.motor.get()+", Velocity: "+v);
                n++;
                info.motor.getEncoder().setPosition(0);
                info.motor.set((n+1)/10.0);
            }
            //System.out.printf("Motor %d: %f\n", i, motorsToRotate[i].motor.getEncoder().getPosition());
        }
        System.out.println(velocities);
        double[] ks = new double[10];
        for (int i=0; i<10; i++) {
            double v = velocities[i];
            ks[i] = v/((i+1)/10.0);
        }
        double sum = 0;
        for (double s : ks) {sum += s;}
        sum /=10;
        return sum;
    }


    @Override
    public boolean isFinished() {
        // System.out.println("lift 0: "+liftSubsystem.getLiftEncoderPosition());
        // System.out.println("lift 1: "+liftSubsystem2.getLiftEncoderPosition());
        
        for (MotorRotationInfo info : motorsToRotate) {
            double pos = info.motor.getEncoder().getPosition();
            if (pos <= info.tickEndpoint + info.initialEncoderVal) {
                info.motor.set(0);
                info.isRunning = false;
            }
        }

        // // Continues the command if any of the motors are running.
        // for (MotorRotationInfo info : motorsToRotate) if (info.isRunning) return false;
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

        public MotorRotationInfo(CANSparkMax motor) {
            this.motor = motor;
        }

        public MotorRotationInfo setRotation(double degrees, double seconds) {
            this.degrees = degrees;
            secondsToRun = seconds;
            tickEndpoint = (DriveSubsystem.ENCODER_TICKS_PER_ROTATION * degrees / 360);
            initialEncoderVal = motor.getEncoder().getPosition();
            reqRPM = (degrees/360) * 60 / secondsToRun;
            return this;
        }

        @Override
        public String toString() {
            return String.format("Degrees: %f, Endpoint: %f, initialEncode: %f, RPM: %f", degrees, tickEndpoint, initialEncoderVal, reqRPM);
        }
    }
    */
}
