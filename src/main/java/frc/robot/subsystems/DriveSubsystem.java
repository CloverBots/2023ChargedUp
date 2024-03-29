package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDs;
import frc.robot.NavXGyro;
import frc.robot.RobotLifecycleCallbacks;

public class DriveSubsystem extends SubsystemBase implements RobotLifecycleCallbacks {

  private static final double LIME_DISTANCE_PID_P = 0.02;
  private static final double LIME_DISTANCE_PID_I = 0.0;
  private static final double LIME_DISTANCE_PID_D = 0.0;

  private static final double LIME_ROTATE_PID_P = 0.01;
  private static final double LIME_ROTATE_PID_I = 0.0;
  private static final double LIME_ROTATE_PID_D = 0.0;
  private static final double LIME_ROTATE_PID_DEFAULT_SETPOINT = 0;

  private static final double DRIVESTRAIGHT_PID_P = 0.8;
  private static final double DRIVESTRAIGHT_PID_I = 0.0;
  private static final double DRIVESTRAIGHT_PID_D = 0.06;

  public static final double DRIVEROTATE_PID_P = 0.0037;
  public static final double DRIVEROTATE_PID_I = 0.00;
  public static final double DRIVEROTATE_PID_D = 0.0;

  public static final double WHEEL_DIAMETER_METERS = 0.1524;
  public static final double ENCODER_POSITION_CONVERSION_FACTOR = 0.1 * WHEEL_DIAMETER_METERS * Math.PI;
  public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = ENCODER_POSITION_CONVERSION_FACTOR * 60.0;
  public static final double ENCODER_TICKS_PER_ROTATION = 2048;

  public final PIDController limeDistancePidController = new PIDController(
      LIME_DISTANCE_PID_P,
      LIME_DISTANCE_PID_I,
      LIME_DISTANCE_PID_D);

  public final PIDController limeRotationPidController = new PIDController(
      LIME_ROTATE_PID_P,
      LIME_ROTATE_PID_I,
      LIME_ROTATE_PID_D);

  public final PIDController driveDistancePidController = new PIDController(
      DRIVESTRAIGHT_PID_P,
      DRIVESTRAIGHT_PID_I,
      DRIVESTRAIGHT_PID_D);

  public final PIDController driveRotatePidController = new PIDController(
      DRIVEROTATE_PID_P,
      DRIVEROTATE_PID_I,
      DRIVEROTATE_PID_D);

  private final TalonFX leftLeadMotor = new TalonFX(IDs.DRIVE_LEFT_LEAD_DEVICE);
  private final TalonFX rightLeadMotor = new TalonFX(IDs.DRIVE_RIGHT_LEAD_DEVICE);
  private final TalonFX leftFollowMotor = new TalonFX(IDs.DRIVE_LEFT_FOLLOW_DEVICE);
  private final TalonFX rightFollowMotor = new TalonFX(IDs.DRIVE_RIGHT_FOLLOW_DEVICE);

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  public final NavXGyro navXGyro = new NavXGyro();

  /**
   * 
   */
  public DriveSubsystem() {
    leftFollowMotor.follow(leftLeadMotor);
    leftFollowMotor.setNeutralMode(NeutralMode.Brake);
    rightFollowMotor.follow(rightLeadMotor);
    rightFollowMotor.setNeutralMode(NeutralMode.Brake);

    leftLeadMotor.setNeutralMode(NeutralMode.Brake);
    rightLeadMotor.setNeutralMode(NeutralMode.Brake);

    rightLeadMotor.setInverted(true);
    rightFollowMotor.setInverted(true);

    leftLeadMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        kPIDLoopIdx,
        kTimeoutMs);
    rightLeadMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        kPIDLoopIdx,
        kTimeoutMs);

    limeRotationPidController.setSetpoint(LIME_ROTATE_PID_DEFAULT_SETPOINT);

    // Temporarily turning saftey off to allow the motors to run without constant
    // updates
    // differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("heading", navXGyro.getHeading());
  }

  public void arcadeDrive(double forward, double rotate) {
    // If forward is 1 or -1, we want to slow down a bit so that we can still turn
    if (forward + rotate > 1.0) {
      forward = forward - rotate;
    } else if (forward - rotate < -1.0) {
      forward = forward + rotate;
    }

    leftLeadMotor.set(TalonFXControlMode.PercentOutput, forward + rotate);
    rightLeadMotor.set(TalonFXControlMode.PercentOutput, forward - rotate);
  }

  // differentialDrive requires constant updates, so we are manually setting the
  // motor speeds to test autonomous
  public void autoDrive(double forward, double rotate) { // TODO: Investigate why arcadeDrive isn't working, but
                                                         // autoDrive is.
    leftLeadMotor.set(TalonFXControlMode.PercentOutput, forward + rotate);
    rightLeadMotor.set(TalonFXControlMode.PercentOutput, forward - rotate);
  }

  // documentation at CTRE Pheonix TalonFX documentation
  public void setOpenLoopRamp(double secondsFromNeutralToFull) {
    leftLeadMotor.configOpenloopRamp(secondsFromNeutralToFull, 10);
    rightLeadMotor.configOpenloopRamp(secondsFromNeutralToFull, 10);
    leftFollowMotor.configOpenloopRamp(secondsFromNeutralToFull, 10);
    rightFollowMotor.configOpenloopRamp(secondsFromNeutralToFull, 10);
  }

  // Get the current position of the left encoder.
  // Currently used to get the left encoder for driving by distance, but may be
  // changed to include right
  public double getLeftEncoderPosition() {
    return leftLeadMotor.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION * ENCODER_POSITION_CONVERSION_FACTOR;
  }

  // Get the current position of the right encoder.
  // Currently used to get the left encoder for driving by distance, but may be
  // changed to include right
  public double getRightEncoderPosition() {
    return rightLeadMotor.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION * ENCODER_POSITION_CONVERSION_FACTOR;
  }

  // Get the average position of left and right encoders.
  public double getAverageEncoderPosition() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
  }

  /**
   * not used
   * private Rotation2d getRotation() {
   * return Rotation2d.fromDegrees(navXGyro.getHeading() % 360.0);
   * }
   */

  public double calculateLimeRotatePidOutput(double measurement) {
    return limeRotationPidController.calculate(measurement);
  }

  public double calculateLimeDistancePidOutput(double measurement) {
    return limeDistancePidController.calculate(measurement);
  }


  public void resetEncoders() {
    leftLeadMotor.setSelectedSensorPosition(0);
    rightLeadMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousInit() {
    setOpenLoopRamp(0);
    // Disable the differentialDrive safety during autonomous
    // differentialDrive requires constant feeding of motor inputs when safety is
    // enabled.
    // differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void teleopInit() {
    setOpenLoopRamp(0.1); // .1 is how much time it takes to get desired value, values of 0.5+ make it
                          // really drifty
    // Re-enable safety for teleop
    // differentialDrive.setSafetyEnabled(true);
  }

  @Override
  public void disabledInit() {
    // TODO Auto-generated method stub
  }
}
