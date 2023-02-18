package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class ArmSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 10;

  private final CANSparkMax motor = new CANSparkMax(IDs.ARM_DEVICE, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = -500.0;

  public static final double UPPER_ENDPOINT = 500; // in rotations

  TimeOfFlight bottomDistanceSensor = new TimeOfFlight(0);
  TimeOfFlight topDistanceSensor = new TimeOfFlight(1);

  public final double BOTTOM_MIN_DISTANCE = 75.0;
  public final double TOP_MIN_DISTANCE = 75.0;
  public ArmSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);
    bottomDistanceSensor.setRangingMode(RangingMode.Short, 33);
    topDistanceSensor.setRangingMode(RangingMode.Short, 33);


    //calibrateArm();

    motor.getEncoder().setPosition(0);

  }
  
  private void calibrateArm() {
    double range = bottomDistanceSensor.getRange();

    if (range <= BOTTOM_MIN_DISTANCE) {
      setArmSpeed(0.2);
      while (range <= BOTTOM_MIN_DISTANCE) {
        range = bottomDistanceSensor.getRange();
      }
      setArmSpeed(0);
      motor.getEncoder().setPosition(0);
    }

    setArmSpeed(-0.2);
    while (range >= BOTTOM_MIN_DISTANCE) {
      range = bottomDistanceSensor.getRange();
    }
    setArmSpeed(0);
    motor.getEncoder().setPosition(0);
      
  }

  public void setArmSpeed(double speed) {
    double range = bottomDistanceSensor.getRange();
    double position = motor.getEncoder().getPosition();

    SmartDashboard.putNumber("Distance Sensor (Arm)", range);
    SmartDashboard.putNumber("Arm Encoder Position", position);
    
    if (bottomDistanceSensor.getRange() <= BOTTOM_MIN_DISTANCE && speed > 0) motor.set(0);
    else if (topDistanceSensor.getRange() <= TOP_MIN_DISTANCE && speed < 0) motor.set(0);

    else motor.set(speed);
  }

  public double getArmEncoderPosition() {
    return -motor.getEncoder().getPosition(); // negative because encoder is weird
  }

  public void setArmMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

  public CANSparkMax getMotor() {
    return motor;
  }
}
