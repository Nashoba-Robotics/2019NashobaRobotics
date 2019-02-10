package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {
    
    public static boolean limelightEnabled = false;
    public static boolean lowGoalSensorEnabled = false;
    public static boolean cargoIntakeSensorEnabled = false;
    public static boolean floorSensorEnabled = false;

    //more sensors, way more sensors

    public static DigitalInput forceSensor1 = new DigitalInput(RobotMap.FORCE_SENSOR_ONE_PORT);
    public static DigitalInput forceSensor2 = new DigitalInput(RobotMap.FORCE_SENSOR_TWO_PORT);

}

