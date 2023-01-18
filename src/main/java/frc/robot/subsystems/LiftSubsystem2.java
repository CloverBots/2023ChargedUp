package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;
import frc.robot.LiftPosition;

public class LiftSubsystem2 extends SubsystemBase implements LiftObserver {
  private final int CURRENT_LIMIT = 30; 

  private final CANSparkMax winch1 = new CANSparkMax(IDs.LIFT_WINCH_DEVICE1, MotorType.kBrushless);

  private LiftPosition liftPosition;

  /**
   * Constructs a new {@link LiftSubsystem} instance.
   */
  public LiftSubsystem2() {
    winch1.setSmartCurrentLimit(CURRENT_LIMIT);

    winch1.setIdleMode(IdleMode.kBrake);

    winch1.setInverted(true);
    liftPosition = LiftPosition.DOWN;
  }

  public void setLiftSpeed(double speed) {
    winch1.set(speed);
  }

  public double getLiftEncoderPosition() {
    return -winch1.getEncoder().getPosition();
  }

  public void setLiftMaximumPosition(double min, double max) {
    winch1.setSoftLimit(SoftLimitDirection.kForward, (float) max);
    winch1.setSoftLimit(SoftLimitDirection.kForward, (float) min);
  }

  /**
   * Returns the position of the lift.
   */
  public LiftPosition getLiftPosition() {
    return liftPosition;
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
}
