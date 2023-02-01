package frc.robot.commands;

import frc.robot.SequentialCommandGroupExtended;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoWaitScoreCommand extends SequentialCommandGroupExtended {
  private final static String SMART_DASHBOARD_AUTO_WAIT_TIME = "Auto Wait Time";
  private final static double COLLISION_SPEED = 0.2; // tune speed
  private final static double TIMEOUT_IN_SECONDS = 2;

  /** Creates a new AutoWaitScore. */
  public AutoWaitScoreCommand(
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      TelescopeSubsystem telescopeSubsystem,
      WristSubsystem wristSubsystem) {

    // Autonomous commands in running order
    addCommands(new SmartDashboardWaitCommand(SMART_DASHBOARD_AUTO_WAIT_TIME));
    addCommands(new DriveToCollisionCommand(driveSubsystem, COLLISION_SPEED, TIMEOUT_IN_SECONDS));
    // TODO arm do thing here
  }
}
