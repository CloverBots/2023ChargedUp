package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {

  TelescopeSubsystem telescopeSubsystem;

  private final DoubleSupplier rightJoystickY;
  public static final double LOWER_ENDPOINT = 0.0;
  private final double APPROACH_MAX_SPEED = 0.2;

  public TelescopeCommand(TelescopeSubsystem telescopeSubsystem, DoubleSupplier rightJoystickY) {
    this.rightJoystickY = rightJoystickY;
    this.telescopeSubsystem = telescopeSubsystem;

    addRequirements(telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Telescope", telescopeSubsystem.getTelescopeEncoderPosition());

    double telescopeSpeed = rightJoystickY.getAsDouble() * .5;
    if (Math.abs(telescopeSpeed) > 0.05) {

      if ((telescopeSubsystem.getTelescopeEncoderPosition() <= LOWER_ENDPOINT && telescopeSpeed > 0) ||
          (telescopeSubsystem.getTelescopeEncoderPosition() >= TelescopeSubsystem.UPPER_ENDPOINT && telescopeSpeed < 0)) {
        telescopeSpeed = 0;
      }

      if (telescopeSubsystem.getTelescopeEncoderPosition() - LOWER_ENDPOINT < 3
          || TelescopeSubsystem.UPPER_ENDPOINT - telescopeSubsystem.getTelescopeEncoderPosition() < 3) {
        telescopeSpeed = Math.min(Math.max(telescopeSpeed, -APPROACH_MAX_SPEED), APPROACH_MAX_SPEED);
      }
      
        telescopeSubsystem.setTelescopeSpeed(telescopeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescopeSubsystem.setTelescopeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
