package frc.robot.commands;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TOFSensorTestCommand extends CommandBase {

    final int MAX = 5;
    private double[] entries = new double[MAX];
    int usedEntries = 0;
    TimeOfFlight timeOfFlight = new TimeOfFlight(0);
    public TOFSensorTestCommand() {
        timeOfFlight.setRangingMode(RangingMode.Short, 33);
        SmartDashboard.putNumber("Sample Time", timeOfFlight.getSampleTime());

    }


    @Override
    public void execute() {
        if (usedEntries < MAX) {
            entries[usedEntries] = timeOfFlight.getRange();
            usedEntries++;
        } else {
            for (int i=1; i<MAX; i++) {
                entries[i-1] = entries[i];
            }
            entries[0] = timeOfFlight.getRange();
        }
        double average = 0;
        for (double value : entries) {
            average += value;
        }
        average /= usedEntries;
        SmartDashboard.putNumber("Distance from Object", average);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}