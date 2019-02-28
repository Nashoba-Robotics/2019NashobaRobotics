package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class LineSensorStrafeCommand extends NRCommand {

    public LineSensorStrafeCommand() {
        super(Drive.getInstance());
    }

    protected void onStart() {
        if (!EnabledSensors.floorSensorOne.get())
            Drive.getInstance().setMotorSpeedInPercent(0, 0, -Drive.getInstance().SENSOR_STRAFE_PERCENT);
        else if (!EnabledSensors.floorSensorThree.get())
            Drive.getInstance().setMotorSpeedInPercent(0, 0, Drive.getInstance().SENSOR_STRAFE_PERCENT);

    }

    protected void onEnd() {
        Drive.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        return !EnabledSensors.floorSensorTwo.get();
    }

}
