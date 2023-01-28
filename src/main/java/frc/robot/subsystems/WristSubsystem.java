package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class WristSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 30; 

  private final CANSparkMax winch1 = new CANSparkMax(IDs.WRIST_DEVICE, MotorType.kBrushless);


  /**
   * Constructs a new {@link WristSubsystem} instance.
   */
  public WristSubsystem() {
    winch1.setSmartCurrentLimit(CURRENT_LIMIT);

    winch1.setIdleMode(IdleMode.kBrake);

    winch1.setInverted(true);
  }

  public void setWristSpeed(double speed) {
    winch1.set(speed);
  }

  public double getWristEncoderPosition() {
    return -winch1.getEncoder().getPosition();
  }

  public void setWristMaximumPosition(double min, double max) {
    winch1.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    winch1.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }


  public Boolean getLowerSwitch() {
    return true;
  }

  public Boolean getUpperSwitch() {
    return true;
  }

  public void resetEncoder() {
    winch1.getEncoder().setPosition(0);
  }

  public CANSparkMax getMotor() {
    return winch1;
  }
}
