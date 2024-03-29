// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class JoystickTrigger extends Trigger {
    private final GenericHID m_joystick;
    private final int m_joystickAxis;
  
    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick The GenericHID object that has the axis (e.g. Joystick, KinectStick, etc)
     * @param axis The axis number (see {@link GenericHID#getRawAxis(int) }
     */
    //public JoystickTrigger(GenericHID joystick, int axis) {
    public JoystickTrigger(GenericHID joystick, int axis) {
      ErrorMessages.requireNonNullParam(joystick, "joystick", "JoystickTrigger");
  
      m_joystick = joystick;
      m_joystickAxis = axis;
    }
  
    /**
     * Gets if the joystick axis is greater than 0.1
     *
     * @return If joystick axis is greater than 0.1
     */

    public boolean get() {
      return m_joystick.getRawAxis(m_joystickAxis) > 0.1 ;

    }
  }