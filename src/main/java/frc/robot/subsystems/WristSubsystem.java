package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class WristSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 30; 

  private final CANSparkMax motor = new CANSparkMax(IDs.WRIST_DEVICE, MotorType.kBrushless);

public static final double UPPER_ENDPOINT = 87; // in rotations

  /**
   * Constructs a new {@link WristSubsystem} instance.
   */
  public WristSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);
  }

  public void setWristSpeed(double speed) {
    motor.set(speed);
  }

  public double getWristEncoderPosition() {
    return motor.getEncoder().getPosition();
  }

  public void setWristMaximumPosition(double min, double max) {
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }


  public Boolean getLowerSwitch() {
    return true;
  }

  public Boolean getUpperSwitch() {
    return true;
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

  public CANSparkMax getMotor() {
    return motor;
  }
}
