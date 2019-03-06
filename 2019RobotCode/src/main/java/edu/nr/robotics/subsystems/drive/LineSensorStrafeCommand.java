package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class LineSensorStrafeCommand extends NRCommand {

    int left = 1;
    int right = 2;

    public static int tracker = 0;

    public LineSensorStrafeCommand() {
        super(Drive.getInstance());
    }

    protected void onStart() {
        if (!EnabledSensors.floorSensorOne.get()) {
            Drive.getInstance().setMotorSpeedInPercent(0, 0, -Drive.getInstance().SENSOR_STRAFE_PERCENT);
            tracker = 1;
        } else if (!EnabledSensors.floorSensorThree.get()) {
            Drive.getInstance().setMotorSpeedInPercent(0, 0, Drive.getInstance().SENSOR_STRAFE_PERCENT);
            tracker = 2;
        } else
            tracker = 0;

    }

    protected void onEnd() {
        Drive.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        return !EnabledSensors.floorSensorThree.get();
    }

}