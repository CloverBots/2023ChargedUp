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
  private final int CURRENT_LIMIT = 20;

  private final CANSparkMax motor = new CANSparkMax(IDs.ARM_DEVICE, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = 0; // 0.0

  public static final double UPPER_ENDPOINT = 76; // 76

  public static final double SLOWDOWN_MARGIN = 15;

  public static final double POWER_AT_ENDPOINT = 0.1;

  TimeOfFlight bottomDistanceSensor = new TimeOfFlight(0);
  TimeOfFlight topDistanceSensor = new TimeOfFlight(1);

  public final double BOTTOM_MIN_DISTANCE = 75.0;
  public final double TOP_MIN_DISTANCE = 75.0;

  public ArmSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(true); //so forward on joystick moves arm up
    
    bottomDistanceSensor.setRangingMode(RangingMode.Short, 33);
    topDistanceSensor.setRangingMode(RangingMode.Short, 33);


    //calibrateArm();

    //motor.getEncoder().setPosition(0);

  }
  /** Not used
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
  */

  public void setArmSpeed(double speed) {

    SmartDashboard.putNumber("Arm Encoder", getArmEncoderPosition());

    if ((getArmEncoderPosition() <= LOWER_ENDPOINT && speed < 0) ||
          (getArmEncoderPosition() >= UPPER_ENDPOINT && speed > 0)) {
        speed = 0;
    }
    final double pos = getArmEncoderPosition();
    
    double adjustedSpeed = calculateAdjustedMotorSpeed(
      pos,
      UPPER_ENDPOINT,
      LOWER_ENDPOINT,
      SLOWDOWN_MARGIN,
      speed,
      POWER_AT_ENDPOINT
    );
    SmartDashboard.putNumber("Adjusted Speed", adjustedSpeed);
    double range = bottomDistanceSensor.getRange();
    
    SmartDashboard.putNumber("Distance Sensor (Arm)", range);
    /**
    if (bottomDistanceSensor.getRange() <= BOTTOM_MIN_DISTANCE && speed > 0) motor.set(0);
    else if (topDistanceSensor.getRange() <= TOP_MIN_DISTANCE && speed < 0) motor.set(0);

    else motor.set(speed);
    */
    motor.set(adjustedSpeed);
  }


  private double calculateAdjustedMotorSpeed(double currentPosition, double upper, double lower, double margin, double currentPower, double powerAtEndpoint) {
    powerAtEndpoint = Math.copySign(powerAtEndpoint, currentPower);
    if (currentPower <= powerAtEndpoint) return currentPower;
    double coefficient = (powerAtEndpoint - currentPower) / Math.pow(margin, 2);
    if (currentPosition >= lower && currentPosition <= lower + margin) {
      return coefficient * Math.pow(currentPosition - (lower + margin), 2) + currentPower;
    }
    else if (currentPosition >= upper-margin && currentPosition <= upper) {
      return coefficient * Math.pow(currentPosition - (upper - margin), 2) + currentPower;
    }
    else return currentPower;
  }

  public double getArmEncoderPosition() {
    return motor.getEncoder().getPosition(); 
  }

  public void setArmMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }
}
