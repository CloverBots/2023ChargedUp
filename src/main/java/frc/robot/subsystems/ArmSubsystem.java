package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class ArmSubsystem extends SubsystemBase {
  private final int CURRENT_LIMIT = 30;

  private final CANSparkMax motor = new CANSparkMax(IDs.ARM_DEVICE, MotorType.kBrushless);

  public ArmSubsystem() {
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(true);

    motor.getEncoder().setPosition(0);
  }

  public void setArmSpeed(double speed) {
    motor.set(speed);
  }

  public double getArmEncoderPosition() {
    return -motor.getEncoder().getPosition();
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
