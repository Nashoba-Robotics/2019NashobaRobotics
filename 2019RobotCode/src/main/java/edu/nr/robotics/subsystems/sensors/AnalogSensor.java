package edu.nr.robotics.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSensor implements Sensor {

    private AnalogInput sensor;
    private int threshold;
    private boolean negate;

    public AnalogSensor(AnalogInput sensor, int threshold) {
        this.sensor = sensor;
        this.threshold = threshold;
        negate = false;
    }

    public AnalogSensor(AnalogInput sensor, int threshold, boolean negate) {
        this.sensor = sensor;
        this.threshold = threshold;
        this.negate = negate;
    }

    public AnalogSensor(int sensorID, int threshold) {
        sensor = new AnalogInput(sensorID);
        this.threshold = threshold;
        negate = false;
    }

    public AnalogSensor(int sensorID, int threshold, boolean negate) {
        sensor = new AnalogInput(sensorID);
        this.threshold = threshold;
        this.negate = negate;
    }

    public AnalogInput getSensor() {
        return sensor;
    }

    public boolean get() {
        if (negate)
            return !(sensor.getValue() > threshold);
        return (sensor.getValue() > threshold);
    }

}
