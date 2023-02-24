// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class IntakeToPositionCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final TelescopeSubsystem telescopeSubsystem;
  private final WristSubsystem wristSubsystem;

  private double armPosition;
  private double armPositionOriginal;
  private double armSpeed;
  private double telescopePosition;
  private double telescopePositionOriginal;
  private double telescopeSpeed;
  private double wristPosition;
  private double wristPositionOriginal;
  private double wristSpeed;

  private int armDirection = 1;
  private int telescopeDirection = 1;
  private int wristDirection = 1;

  private boolean armDone;
  private boolean telescopeDone;
  private boolean wristDone;

  private boolean secondStage;
  private double secondPosition;
  private int secondSystem = 0;// 1= arm, 2 = telescope, 3 = wrist

  /** Creates a new IntakeToPosition. */
  public IntakeToPositionCommand(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem,
      WristSubsystem wristSubsystem, double armPosition, double armSpeed,
      double telescopePosition, double telescopeSpeed, double wristPosition, double wristSpeed,
      double secondPosition, int secondSystem) {
    //

    if (armPosition > ArmSubsystem.UPPER_ENDPOINT) {
      armPosition = ArmSubsystem.UPPER_ENDPOINT - 1; // -1 because you can overshoot by just a bit
    }

    if (armPosition < ArmSubsystem.LOWER_ENDPOINT) {
      armPosition = ArmSubsystem.LOWER_ENDPOINT + 1; // 1 because you can overshoot by just a bit
    }

    if (telescopePosition > TelescopeSubsystem.UPPER_ENDPOINT) {
      telescopePosition = TelescopeSubsystem.UPPER_ENDPOINT - 1; // -1 because you can overshoot by just a bit
    }

    if (telescopePosition < TelescopeSubsystem.LOWER_ENDPOINT) {
      telescopePosition = TelescopeSubsystem.LOWER_ENDPOINT + 1; // 1 because you can overshoot by just a bit
    }

    if (wristPosition > WristSubsystem.UPPER_ENDPOINT) {
      wristPosition = WristSubsystem.UPPER_ENDPOINT - 1; // -1 because you can overshoot by just a bit
    }

    if (wristPosition < WristSubsystem.LOWER_ENDPOINT) {
      wristPosition = WristSubsystem.LOWER_ENDPOINT + 1; // 1 because you can overshoot by just a bit
    }

    this.armSubsystem = armSubsystem;
    this.telescopeSubsystem = telescopeSubsystem;
    this.wristSubsystem = wristSubsystem;

    this.armPosition = armPosition;
    this.armPositionOriginal = armPosition;
    this.armSpeed = armSpeed;
    this.telescopePosition = telescopePosition;
    this.telescopePositionOriginal = telescopePosition;
    this.telescopeSpeed = telescopeSpeed;
    this.wristPosition = wristPosition;
    this.wristPositionOriginal = wristPosition;
    this.wristSpeed = wristSpeed;
    this.secondPosition = secondPosition;
    this.secondSystem = secondSystem;

    addRequirements(armSubsystem);
    addRequirements(wristSubsystem);
    addRequirements(telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset from earlier uses of this command
    armDirection = 1;
    telescopeDirection = 1;
    wristDirection = 1;
    armDone = false;
    telescopeDone = false;
    wristDone = false;
    secondStage = false;

    armPosition = armPositionOriginal;
    telescopePosition = telescopePositionOriginal;
    wristPosition = wristPositionOriginal;

    // In determining direction, the encoder values that it stops at are
    // not exact so allow a tolerance (that's where the 1.5 comes from).
    if (armSubsystem.getArmEncoderPosition() - armPosition > 1.5) {
      armDirection = -1;
    }
    if (armSubsystem.getArmEncoderPosition() > armPosition) {
      armDirection = -1;
    }

    if (telescopeSubsystem.getTelescopeEncoderPosition() - telescopePosition > 1.5) {
      telescopeDirection = -1; // retract telescope
    }

    if (wristSubsystem.getWristEncoderPosition() - wristPosition > 1.5) {
      wristDirection = -1;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If arm going up then do arm movement to completion and then telescope & wrist
    if (armDirection == 1) {
      if (!armDone) {
        armSubsystem.setArmSpeed(armSpeed * armDirection);
      } else {
        telescopeSubsystem.setTelescopeSpeed(telescopeSpeed * telescopeDirection, false);
        wristSubsystem.setWristSpeed(wristSpeed * wristDirection);
      }
    } else {
      // If arm going down then retract telescope first and then arm and wrist
      if (!telescopeDone) {
        telescopeSubsystem.setTelescopeSpeed(telescopeSpeed * telescopeDirection, false);
      } else {
        armSubsystem.setArmSpeed(armSpeed * armDirection);
        wristSubsystem.setWristSpeed(wristSpeed * wristDirection);
      }
    }

    if (secondStage) {
      if (secondSystem == 3) {
        wristSubsystem.setWristSpeed(wristSpeed * wristDirection);
      } else if (secondSystem == 2) {
        telescopeSubsystem.setTelescopeSpeed(telescopeSpeed * telescopeDirection, false);
      } else if (secondSystem == 1) {
        armSubsystem.setArmSpeed(armSpeed * armDirection);
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmSpeed(0);
    telescopeSubsystem.setTelescopeSpeed(0, false);
    wristSubsystem.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (armDirection == -1 && armSubsystem.getArmEncoderPosition() <= armPosition) {
      armDone = true;
      armSubsystem.setArmSpeed(0);
    } else if (armDirection == 1 && armSubsystem.getArmEncoderPosition() >= armPosition) {
      armDone = true;
      armSubsystem.setArmSpeed(0);
    }

    if (telescopeDirection == -1 && telescopeSubsystem.getTelescopeEncoderPosition() <= telescopePosition) {
      telescopeDone = true;
      telescopeSubsystem.setTelescopeSpeed(0, false);
    } else if (telescopeDirection == 1 && telescopeSubsystem.getTelescopeEncoderPosition() >= telescopePosition) {
      telescopeDone = true;
      telescopeSubsystem.setTelescopeSpeed(0, false);
    }

    if (wristDirection == -1 && wristSubsystem.getWristEncoderPosition() <= wristPosition) {
      wristDone = true;
      wristSubsystem.setWristSpeed(0);
    } else if (wristDirection == 1 && wristSubsystem.getWristEncoderPosition() >= wristPosition) {
      wristDone = true;
      wristSubsystem.setWristSpeed(0);
    }

    // System.out.println("arm: " + armDone + ", telescope: " + telescopeDone + ",
    // wrist: " + wristDone);

    if (armDirection == -1 && armDone && wristDone && telescopeDone) {
      // arm moving down, done
      return true;
    } else if (!secondStage && armDirection == 1 && armDone && wristDone && telescopeDone) {
      // arm moving up and ready for second stage
      secondStage = true;
      if (secondSystem == 3) {
        wristDone = false;
        wristDirection = 1;
        wristPosition = secondPosition;
        if (wristSubsystem.getWristEncoderPosition() - wristPosition > 1.5) {
          wristDirection = -1;
        }
        System.out.println("wristPos: " + wristPosition + ", direction: " + wristDirection);
      } else if (secondSystem == 2) {
        telescopeDone = false;
        telescopeDirection = 1;
        telescopePosition = secondPosition;
        if (telescopeSubsystem.getTelescopeEncoderPosition() - telescopePosition > 1.5) {
          telescopeDirection = -1;
        }
      } else if (secondSystem == 1) {
        armDone = false;
        armDirection = 1;
        armPosition = secondPosition;
        if (armSubsystem.getArmEncoderPosition() - armPosition > 1.5) {
          armDirection = -1;
        }
      }
      return false;
    } else if (secondStage && armDirection == 1 && armDone && wristDone && telescopeDone) {
      // arm moving up and second stage is done so all done
      return true;
    } else {
      return false;
    }
  }
}
