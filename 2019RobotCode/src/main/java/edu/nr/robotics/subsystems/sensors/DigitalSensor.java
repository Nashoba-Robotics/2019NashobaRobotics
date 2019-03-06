package edu.nr.robotics.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSensor implements Sensor {

    private DigitalInput sensor;

    public DigitalSensor(DigitalInput sensor) {
        this.sensor = sensor;
    }

    public DigitalSensor(int sensorID) {
        sensor = new DigitalInput(sensorID);
    }

    public DigitalInput getSensor() {
        return sensor;
    }

	@Override
	public boolean get() {
		return sensor.get();
	}
}
