package edu.nr.robotics.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorVoting {
    boolean in1, in2, in3;

    public SensorVoting(DigitalInput sensor1, DigitalInput sensor2, DigitalInput sensor3) {
        in1 = sensor1.get();
        in2 = sensor2.get();
        in3 = sensor3.get();
    }

    public boolean isTrue() {
        return ((in1 && in2) || (in1 && in3) || (in2 && in3));
    }

}