package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;
import frc.robot.RobotContainer;

public class WristSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 10; 

  private final CANSparkMax motor = new CANSparkMax(IDs.WRIST_DEVICE, MotorType.kBrushless);

  public static final double LOWER_ENDPOINT = -40; //0

public static final double UPPER_ENDPOINT = 54; // 66
  
  private static final double MARGIN = 15;

  /**
   * Constructs a new {@link WristSubsystem} instance.
   */
  public WristSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    //setWristMaximumPosition(LOWER_ENDPOINT, UPPER_ENDPOINT);

    motor.setInverted(false);
    
  }

  public void setWristSpeed(double speed) {

    SmartDashboard.putNumber("Wrist Encoder", getWristEncoderPosition());
    
    if ((getWristEncoderPosition() <= LOWER_ENDPOINT && speed < 0) ||
          (getWristEncoderPosition() >= UPPER_ENDPOINT && speed > 0)) {
        speed = 0;
      }


    double adjustedSpeed = RobotContainer.calculateAdjustedMotorSpeed(
      getWristEncoderPosition(),
      UPPER_ENDPOINT,
      LOWER_ENDPOINT,
      MARGIN,
      speed,
      0.1
    );
    motor.set(adjustedSpeed);
  }

  public double getWristEncoderPosition() {
    return motor.getEncoder().getPosition(); 
  }

  public void setWristMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) min);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }
}
