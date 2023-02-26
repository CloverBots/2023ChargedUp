package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private DoubleSupplier inSpeed;
    private DoubleSupplier outSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier inSpeed, DoubleSupplier outSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.inSpeed = inSpeed;
        this.outSpeed = outSpeed;
        // this.negativeIntakeSpeed = negativeIntakeSpeed;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void execute() {
        double in = inSpeed.getAsDouble();
        double out = outSpeed.getAsDouble();
        double speed = 0;

        if (in > 0.05 && out < 0.05) {
            speed = in;
        } else if (in < 0.05 && out > 0.05) {
            speed = -out * 0.3;
        } else {
            speed = 0;
        }

        intakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
