package edu.nr.robotics.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSensor {

    private AnalogInput sensor;
    private int threshold;

    public AnalogSensor(AnalogInput sensor, int threshold) {
        this.sensor = sensor;
        this.threshold = threshold;
    }
    public AnalogSensor(int sensorID, int threshold) {
        sensor = new AnalogInput(sensorID);
        this.threshold = threshold;
    }

    public AnalogInput getSensor() {
        return sensor;
    }

    public boolean get() {
        return !(sensor.getValue() > threshold);
    }

}
