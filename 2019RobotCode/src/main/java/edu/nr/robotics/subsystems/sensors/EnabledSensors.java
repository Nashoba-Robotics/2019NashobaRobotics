package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;

public class EnabledSensors {
    
    public static boolean limelightEnabled = false;
    public static boolean lowGoalSensorEnabled = false;
    public static boolean cargoIntakeSensorEnabled = false;
    public static boolean floorSensorEnabled = false;

    //more sensors, way more sensors

    public static DigitalSensor elevatorSensor = new DigitalSensor(RobotMap.ELEVATOR_SENSOR_PORT);
    //public static DigitalInput elevatorSensor2 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_2);
    //public static DigitalInput elevatorSensor3 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_3);
    //public static DigitalInput elevatorSensor4 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_4);

    //public static DigitalInput cargoIntakeSensorOne = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_1);
    //public static DigitalInput cargoIntakeSensorTwo = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_2);
    //public static DigitalInput cargoIntakeSensorThree = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_3);

    public static AnalogSensor floorSensorOne = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_1, Drive.LINE_SENSOR_THRESHOLD);
    public static DigitalSensor floorSensorTwo = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_2);
    public static AnalogSensor floorSensorThree = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_3, Drive.LINE_SENSOR_THRESHOLD);
    public static DigitalSensor floorSensorFour = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_4);
    public static AnalogSensor floorSensorFive = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_5, Drive.LINE_SENSOR_THRESHOLD);

    public static DigitalSensor forceSensorOne = new DigitalSensor(RobotMap.FORCE_SENSOR_1);
    public static DigitalSensor forceSensorTwo = new DigitalSensor(RobotMap.FORCE_SENSOR_2);
    //public static DigitalSensor forceSensorThree = new DigitalSensor(RobotMap.FORCE_SENSOR_3);

}

