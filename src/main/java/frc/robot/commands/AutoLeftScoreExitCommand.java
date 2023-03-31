package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SequentialCommandGroupExtended;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoLeftScoreExitCommand extends AutoScoreExitBaseCommand {


  /** Creates a new AutoScoreExit. */
  public AutoLeftScoreExitCommand(
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      TelescopeSubsystem telescopeSubsystem,
      WristSubsystem wristSubsystem) {

    super(armSubsystem, driveSubsystem, intakeSubsystem, telescopeSubsystem, wristSubsystem);
    
    addCommands(new SpinToAngleCommand(driveSubsystem, -145, -.2));
  }
}
