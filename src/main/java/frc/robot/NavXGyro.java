package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class NavXGyro {
    private final AHRS ahrs = new AHRS(IDs.AHRS_PORT_ID);

    public void setup() {
        ahrs.calibrate();
        ahrs.reset();
    }

    public double getHeading() {
        return ahrs.getYaw();
    }
   /**
    * This HIGHLY depends on the orientation of the RoboRIO!!! 
    * You may have to use getPitch() instead, depending on its orientation.
    */
    public double getRoll() {
        return ahrs.getRoll();
    }

    public void reset() {
        ahrs.reset();
    }
}
