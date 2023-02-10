package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {

  TelescopeSubsystem telescopeSubsystem;

  public static final double UPPER_ENDPOINT = 87;
  public static final double LOWER_ENDPOINT = 0;

  public TelescopeCommand(TelescopeSubsystem telescopeSubsystem) {
    this.telescopeSubsystem = telescopeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
