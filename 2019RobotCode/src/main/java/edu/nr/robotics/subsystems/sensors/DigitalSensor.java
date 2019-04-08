package edu.nr.robotics.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSensor implements Sensor {

    private DigitalInput sensor;
    private boolean negate;

    public DigitalSensor(DigitalInput sensor) {
        this.sensor = sensor;
        negate = false;
    }

    public DigitalSensor (DigitalInput sensor, boolean negate) {
        this.sensor = sensor;
        this.negate = negate;

    }

    public DigitalSensor(int sensorID) {
        sensor = new DigitalInput(sensorID);
        negate = false;
    }

    public DigitalSensor(int sensorID, boolean negate) {
        sensor = new DigitalInput(sensorID);
        this.negate = negate;
    }

    public DigitalInput getSensor() {
        return sensor;
    }

	@Override
	public boolean get() {
        if (negate) {
            return !sensor.get();
        }
		return sensor.get();
	}
}
