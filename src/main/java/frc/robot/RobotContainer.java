// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.VisionTargetTracker.LedMode;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoScoreChargeCommand;
import frc.robot.commands.AutoScoreExitCommand;
import frc.robot.commands.AutoWaitScoreCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.DriveToCollisionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeToPositionCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double VISION_TARGET_HEIGHT = 34; // inches
  private static final double CAMERA_HEIGHT = 21.25;
  private static final double CAMERA_PITCH = 0; // degrees

  private static final double speed = -.1;
  private static final double timeoutInSeconds = 3;

  private final VisionConfiguration visionConfiguration = new VisionConfiguration(
      VISION_TARGET_HEIGHT,
      CAMERA_HEIGHT,
      CAMERA_PITCH);

  private final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(visionConfiguration);

  private final XboxController driverController = new XboxController(IDs.CONTROLLER_DRIVE_PORT);
  private final XboxController operatorController = new XboxController(IDs.CONTROLLER_OPERATOR_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();

  private final ArmCommand armCommand = new ArmCommand(armSubsystem, driverController::getLeftY);
  private final WristCommand wristCommand = new WristCommand(wristSubsystem, driverController::getRightY);
  private final TelescopeCommand telescopeCommand = new TelescopeCommand(telescopeSubsystem, operatorController::getLeftY);    

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, operatorController::getRightY);
/**
  private final DriveFromControllerCommand driveFromController = new DriveFromControllerCommand(
      driveSubsystem,
      driverController::getLeftY,
      driverController::getRightX,
      driverController::getLeftTriggerAxis);
*/
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
 //   driveSubsystem.setDefaultCommand(driveFromController);
    armSubsystem.setDefaultCommand(armCommand);
    wristSubsystem.setDefaultCommand(wristCommand);
    telescopeSubsystem.setDefaultCommand(telescopeCommand);
    intakeSubsystem.setDefaultCommand(intakeCommand);

    configureTriggerBindings();
    configureChooserModes();

    visionTargetTracker.setLedMode(LedMode.FORCE_ON);
    
    SmartDashboard.putNumber("Auto Distance Inches", 270);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   */
  private void configureTriggerBindings() {

    JoystickTrigger startIntakeTrigger = new JoystickTrigger(operatorController,
        XboxController.Axis.kRightTrigger.value);
   // startIntakeTrigger.whileTrue(new IntakeCommand(intakeSubsystem, operatorController::getRightTriggerAxis, operatorController::getRightTriggerAxis));

    // JoystickButton limeLightTestButton = new JoystickButton(operatorController,
    // XboxController.Button.kA.value);
    // limeLightTestButton.whileHeld(new LimeLightTestCommand(visionTargetTracker));

    JoystickButton balance = new JoystickButton(driverController, XboxController.Button.kB.value);
    balance.whileTrue(new AutoBalanceCommand(driveSubsystem));

   // JoystickButton driveToCollisionButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
   // driveToCollisionButton.onFalse(new DriveToCollisionCommand(driveSubsystem, speed, timeoutInSeconds));

   // JoystickButton alignButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
   // alignButton.whileTrue(new AutoAlignCommand(driveSubsystem, visionTargetTracker, 2));

    //JoystickButton intakeToPositionHighButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    //intakeToPositionHighButton.onTrue(new IntakeToPositionCommand(armSubsystem, telescopeSubsystem, wristSubsystem, 30, .5, 60, .2, 60, .7));

  }

  private void configureChooserModes() {

    SmartDashboard.putData("Autonomous Mode", chooser);
    SmartDashboard.putNumber("Auto Wait Time", 0);
/**
    chooser.setDefaultOption("AutoScoreChargeCommand", new AutoScoreChargeCommand(
        armSubsystem,
        driveSubsystem,
        intakeSubsystem,
        telescopeSubsystem,
        wristSubsystem));
    chooser.addOption("AutoScoreExitCommand", new AutoScoreExitCommand(
        armSubsystem,
        driveSubsystem,
        intakeSubsystem,
        telescopeSubsystem,
        wristSubsystem));
    chooser.addOption("AutoWaitScore", new AutoWaitScoreCommand(
        armSubsystem,
        driveSubsystem,
        intakeSubsystem,
        telescopeSubsystem,
        wristSubsystem));
        */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
