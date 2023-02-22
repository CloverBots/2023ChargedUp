package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private DoubleSupplier intakeSpeed;
    private DoubleSupplier negativeIntakeSpeed;
    
    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeSpeed ) { // DoubleSupplier negativeIntakeSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;
        //this.negativeIntakeSpeed = negativeIntakeSpeed;
        addRequirements(intakeSubsystem);
        
    }

    @Override
    public void execute() {
        //double speed = intakeSpeed.getAsDouble() >= 0.1 ? intakeSpeed.getAsDouble() : // fall back to reverse if not outside deadzone
        //(negativeIntakeSpeed.getAsDouble() >= 0.1 ? -negativeIntakeSpeed.getAsDouble() : 0); // negative reverse trigger speed if outside deadzone
        double speed = intakeSpeed.getAsDouble();
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
