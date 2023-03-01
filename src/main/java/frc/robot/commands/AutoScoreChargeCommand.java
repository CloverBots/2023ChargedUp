package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SequentialCommandGroupExtended;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoScoreChargeCommand extends SequentialCommandGroupExtended {
  private final static double DRIVE_SPEED = .5; // tune both speeds
  private final static double DISTANCE = -2.73; // -2.75, -4

  /** Creates a new AutoScoreCharge. */
  public AutoScoreChargeCommand(
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      TelescopeSubsystem telescopeSubsystem,
      WristSubsystem wristSubsystem) {

    // Autonomous commands in running order

    addCommands(new IntakeToPositionCommand(armSubsystem, telescopeSubsystem,
        wristSubsystem,
        64, 0.5, // 67
        190, 1.0, // 190
        40, 0.3, // 40
        10, 3)); // 10

    addInstant(() -> intakeSubsystem.setIntakeSpeed(-0.2), intakeSubsystem);
    addCommands(new WaitCommand(0.5)); // 1 second
    addInstant(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem);

    addCommands(new IntakeToPositionCommand(armSubsystem, telescopeSubsystem, wristSubsystem,
        0, 0.5, // 0
        0, 1.0, // 0
        0, 0.3, // 0
        0, 3)); // 0)

    addCommands(new DriveToDistanceCommand(driveSubsystem, DISTANCE, DRIVE_SPEED, 0, 0.1));

    // addCommands(new AutoBalanceCommand(driveSubsystem));
  }
}
