// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.VisionTargetTracker.LedMode;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoScoreChargeCommand;
import frc.robot.commands.AutoScoreExitCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeToPositionCommand;
import frc.robot.commands.TelescopeBypassSafetyCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
  // operatorController::getLeftTriggerAxis
  private final ArmCommand armCommand = new ArmCommand(armSubsystem, operatorController::getLeftY);
  private final WristCommand wristCommand = new WristCommand(wristSubsystem, operatorController::getRightY);
  private final TelescopeBypassSafetyCommand telescopeBypassSafetyCommand = new TelescopeBypassSafetyCommand(
      telescopeSubsystem);
    private final TelescopeCommand telescopeInCommand = new TelescopeCommand(telescopeSubsystem, true, false);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem,
      operatorController::getRightTriggerAxis, operatorController::getLeftTriggerAxis);

  private final IntakeToPositionCommand intakeToHighPositionCommand = new IntakeToPositionCommand(armSubsystem,
      telescopeSubsystem, wristSubsystem,
      64, 0.7, // 67
      95, 0.8, // 190
      40, 0.3, // 40
      10, 3); // 10

  private final IntakeToPositionCommand intakeToMiddlePositionCommand = new IntakeToPositionCommand(armSubsystem,
      telescopeSubsystem, wristSubsystem,
      55, 0.5, // 55
      26, 0.5, // 50
      40, 1.0, // 30
      10, 3); // 5

  private final IntakeToPositionCommand intakeToHumanPositionCommand = new IntakeToPositionCommand(armSubsystem,
      telescopeSubsystem, wristSubsystem,
      57, 0.5, // 65
      0, 0.6, // 0
      10, 0.3, // 0
      10, 3); // 0

  private final IntakeToPositionCommand intakeToHomePositionCommand = new IntakeToPositionCommand(armSubsystem,
      telescopeSubsystem, wristSubsystem,
      0, 0.3, // 0
      0, 0.8, // 0
      0, 0.3, // 0
      0, 3); // 0

  private final IntakeToPositionCommand intakeToGroundPositionCommand = new IntakeToPositionCommand(armSubsystem,
      telescopeSubsystem, wristSubsystem,
      30, 0.7, // 30
      140, 1.0, // 140
      -30, 0.5, // -30
      15, 1); // 15

  private final DriveFromControllerCommand driveFromController = new DriveFromControllerCommand(
      driveSubsystem,
      driverController::getLeftY,
      driverController::getRightX,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis);

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveFromController);
    armSubsystem.setDefaultCommand(armCommand);
    wristSubsystem.setDefaultCommand(wristCommand);
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

    // JoystickButton limeLightTestButton = new JoystickButton(operatorController,
    // XboxController.Button.kA.value);
    // limeLightTestButton.whileHeld(new LimeLightTestCommand(visionTargetTracker));

    JoystickButton balance = new JoystickButton(driverController, XboxController.Button.kB.value);
    balance.whileTrue(new AutoBalanceCommand(driveSubsystem));

    JoystickButton telescopeInButton = new JoystickButton(operatorController, Button.kRightBumper.value);
    telescopeInButton.whileTrue(telescopeInCommand);

    JoystickButton telescopeOutButton = new JoystickButton(operatorController, Button.kLeftBumper.value);
    telescopeOutButton.whileTrue(new TelescopeCommand(telescopeSubsystem, false, true));

    // JoystickButton driveToCollisionButton = new JoystickButton(driverController,
    // XboxController.Button.kY.value);
    // driveToCollisionButton.onFalse(new DriveToCollisionCommand(driveSubsystem,
    // speed, timeoutInSeconds));

    // JoystickButton alignButton = new JoystickButton(driverController,
    // XboxController.Button.kA.value);
    // alignButton.whileTrue(new AutoAlignCommand(driveSubsystem,
    // visionTargetTracker, 2));

    JoystickButton intakeToPositionHighButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    intakeToPositionHighButton.onTrue(intakeToHighPositionCommand);

    JoystickButton intakeToPositionMiddleButton = new JoystickButton(operatorController,
        XboxController.Button.kB.value);
    intakeToPositionMiddleButton.onTrue(intakeToMiddlePositionCommand);

    JoystickButton intakeToPositionHumanButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    intakeToPositionHumanButton.onTrue(intakeToHumanPositionCommand);

    POVButton intakeToPositionHomeButton = new POVButton(operatorController, 180); // Down
    intakeToPositionHomeButton.onTrue(intakeToHomePositionCommand);

    JoystickButton intakeToPositionGroundButton = new JoystickButton(operatorController,
        XboxController.Button.kA.value);
    intakeToPositionGroundButton.onTrue(intakeToGroundPositionCommand);

    // Provide ability to bypass the encoder safety limits in order to retract the
    // telescope after a power interruption
    POVButton dPadDownButton = new POVButton(driverController, 180); // down
    dPadDownButton.onTrue(telescopeBypassSafetyCommand);

  }

  public static double calculateAdjustedMotorSpeed(
    double currentPosition,
    double upper,
    double lower,
    double margin,
    double currentPower,
    double powerAtEndpoint) {
    if (Math.abs(currentPower) <= powerAtEndpoint) return currentPower;
    powerAtEndpoint = Math.copySign(powerAtEndpoint, currentPower);
    double coefficient = (powerAtEndpoint - currentPower) / Math.pow(margin, 2);
    if (currentPosition >= lower && currentPosition <= lower + margin) {
      return coefficient * Math.pow(currentPosition - (lower + margin), 2) + currentPower;
    }
    else if (currentPosition >= upper-margin && currentPosition <= upper) {
      return coefficient * Math.pow(currentPosition - (upper - margin), 2) + currentPower;
    }
    else return currentPower;
  }

  private void configureChooserModes() {

    SmartDashboard.putData("Autonomous Mode", chooser);
    SmartDashboard.putNumber("Auto Wait Time", 0);

    chooser.addOption("AutoScoreChargeCommand", new AutoScoreChargeCommand(
        armSubsystem,
        driveSubsystem,
        intakeSubsystem,
        telescopeSubsystem,
        wristSubsystem));
    chooser.setDefaultOption("AutoScoreExitCommand", new AutoScoreExitCommand(
        armSubsystem,
        driveSubsystem,
        intakeSubsystem,
        telescopeSubsystem,
        wristSubsystem));

  }

  /**
   * @return
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
