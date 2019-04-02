package edu.nr.robotics.subsystems.sensors;

public class SensorVoting {
    boolean in1, in2, in3;

    public SensorVoting(Sensor sensor1, Sensor sensor2, Sensor sensor3) {
        in1 = sensor1.get();
        in2 = sensor2.get();
        in3 = sensor3.get();
    }

    public SensorVoting(boolean in1, boolean in2, boolean in3) {
        this.in1 = in1;
        this.in2 = in2;
        this.in3 = in3; 
    }

    public boolean isTrue() {
        return ((in1 && in2) || (in1 && in3) || (in2 && in3));
    }

}