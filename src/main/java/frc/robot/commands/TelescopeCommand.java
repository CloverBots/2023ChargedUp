package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {
  private static final int INCREMENT = 5;
  TelescopeSubsystem telescopeSubsystem;

  private final boolean rightBumper;
  private final boolean leftBumper;
  double startingPosition;

  private static final double SPEED = 0.25;

  public TelescopeCommand(TelescopeSubsystem telescopeSubsystem, boolean rightBumper,
      boolean leftBumper) {
    this.rightBumper = rightBumper;
    this.leftBumper = leftBumper;
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

    if (rightBumper && !leftBumper) {

      telescopeSubsystem.setTelescopeSpeed(-SPEED, false);
    } else if (!rightBumper && leftBumper) {
      telescopeSubsystem.setTelescopeSpeed(SPEED, false);
    } else {
      telescopeSubsystem.setTelescopeSpeed(0, false);

    }

  }

  @Override
  public void end(boolean interrupted) {
    //telescopeSubsystem.setTelescopeSpeed(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // if (rightBumper && telescopeSubsystem.getTelescopeEncoderPosition() <= startingPosition - INCREMENT) {
    //   return true;
    // } else if (leftBumper && telescopeSubsystem.getTelescopeEncoderPosition() >= startingPosition + INCREMENT) {
    //   return true;
    // } else {
    //   return true;
    // }
  }
}
