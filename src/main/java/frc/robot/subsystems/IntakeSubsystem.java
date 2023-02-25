package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;

public class IntakeSubsystem extends SubsystemBase {
    private final int CURRENT_LIMIT = 30;

    private final CANSparkMax motor = new CANSparkMax(IDs.INTAKE_DEVICE, MotorType.kBrushless);

    /**
     * Constructs a new {@link IntakeSubsystem} instance.
     */
    public IntakeSubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        motor.setIdleMode(IdleMode.kBrake);

        motor.setInverted(false);
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    public double getIntakeEncoderPosition() {
        return motor.getEncoder().getPosition(); 
    }

    public void setIntakeMaximumPosition(double min, double max) {
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }
}
