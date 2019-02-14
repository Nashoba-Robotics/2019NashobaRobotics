package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {
    
    public static boolean limelightEnabled = false;
    public static boolean lowGoalSensorEnabled = false;
    public static boolean cargoIntakeSensorEnabled = false;
    public static boolean floorSensorEnabled = false;

    //more sensors, way more sensors

    public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);

    public static DigitalInput cargoIntakeSensorOne = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_1);
    public static DigitalInput cargoIntakeSensorTwo = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_2);
    public static DigitalInput cargoIntakeSensorThree = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_3);

    public static DigitalInput floorSensorOne = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_1);
    public static DigitalInput floorSensorTwo = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_2);
    public static DigitalInput floorSensorThree = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_3);
    public static DigitalInput floorSensorFour = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_4);
    public static DigitalInput floorSensorFive = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_5);

    public static DigitalInput forceSensorOne = new DigitalInput(RobotMap.FORCE_SENSOR_1);
    public static DigitalInput forceSensorTwo = new DigitalInput(RobotMap.FORCE_SENSOR_2);

}

