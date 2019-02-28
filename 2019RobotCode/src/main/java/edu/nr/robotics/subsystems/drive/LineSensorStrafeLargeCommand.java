package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class LineSensorStrafeLargeCommand extends NRCommand {

    private double percent;

    public LineSensorStrafeLargeCommand(double percent) {
        super(Drive.getInstance());
        this.percent = percent;
    }

    protected void onStart() {
        Drive.getInstance().setMotorSpeedInPercent(0, 0, percent);
    }

    protected void onEnd() {
        Drive.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        return !EnabledSensors.floorSensorOne.get() || !EnabledSensors.floorSensorTwo.get() || !EnabledSensors.floorSensorThree.get();
    }

}
