package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;

public class LineSensorStrafeCommand extends NRCommand {

    int left = 1;
    int right = 2;

    public static boolean sensorStart;

    public static int tracker = 0;

    public LineSensorStrafeCommand() {
        super(Drive.getInstance());
    }

    protected void onStart() {
        sensorStart = !(new SensorVoting(EnabledSensors.floorSensorTwo, EnabledSensors.floorSensorThree, EnabledSensors.floorSensorFour).isTrue());

        if (!sensorStart) {
            if (!EnabledSensors.floorSensorOne.get() || !EnabledSensors.floorSensorTwo.get()) {
                Drive.getInstance().setMotorSpeedInPercent(0, 0, -Drive.getInstance().SENSOR_STRAFE_PERCENT);
                tracker = 1;
            } else if (!EnabledSensors.floorSensorFour.get() || !EnabledSensors.floorSensorFive.get()) {
                Drive.getInstance().setMotorSpeedInPercent(0, 0, Drive.getInstance().SENSOR_STRAFE_PERCENT);
                tracker = 2;
            } else {
                tracker = 0;
            }
        }

    }

    protected void onEnd() {
        Drive.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        return !(new SensorVoting(EnabledSensors.floorSensorTwo, EnabledSensors.floorSensorThree, EnabledSensors.floorSensorFour).isTrue());
    }

}