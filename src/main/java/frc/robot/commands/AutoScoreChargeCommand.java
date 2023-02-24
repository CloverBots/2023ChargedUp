package frc.robot.commands;

import frc.robot.SequentialCommandGroupExtended;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoScoreChargeCommand extends SequentialCommandGroupExtended {
  private final static double COLLISION_SPEED = .2;
  private final static double TIMEOUT_IN_SECONDS = 2;
  private final static double DRIVE_SPEED = .2; // tune both speeds
  private final static double DISTANCE = 4; // measure distance

  /** Creates a new AutoScoreCharge. */
  public AutoScoreChargeCommand(
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      TelescopeSubsystem telescopeSubsystem,
      WristSubsystem wristSubsystem) {

    // Autonomous commands in running order
    //addCommands(new DriveToCollisionCommand(driveSubsystem, COLLISION_SPEED, TIMEOUT_IN_SECONDS));
    // TODO arm do thing here
    addCommands(new DriveToDistanceCommand(driveSubsystem, DISTANCE, DRIVE_SPEED, 0, 0.1));
    //addCommands(new AutoBalanceCommand(driveSubsystem));
  }
}
