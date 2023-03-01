package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeBypassSafetyCommand extends CommandBase {

  private static final double SPEED = -0.3;
  private static final int INCREMENT = 5;
  TelescopeSubsystem telescopeSubsystem;
  double startingPosition;

  public TelescopeBypassSafetyCommand(TelescopeSubsystem telescopeSubsystem) {
    this.telescopeSubsystem = telescopeSubsystem;

    addRequirements(telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = telescopeSubsystem.getTelescopeEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescopeSubsystem.setTelescopeSpeed(SPEED, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescopeSubsystem.setTelescopeSpeed(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (telescopeSubsystem.getTelescopeEncoderPosition() <= startingPosition - INCREMENT) {
      return true;
    } else {
      return false;
    }
  }
}
