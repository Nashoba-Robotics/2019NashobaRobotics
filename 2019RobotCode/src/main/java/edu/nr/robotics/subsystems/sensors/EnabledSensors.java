package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {
    
    public static boolean limelightEnabled = false;
    public static boolean lowGoalSensorEnabled = false;
    public static boolean cargoIntakeSensorEnabled = false;
    public static boolean floorSensorEnabled = false;

    //more sensors, way more sensors

    public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);
    //public static DigitalInput elevatorSensor2 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_2);
    //public static DigitalInput elevatorSensor3 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_3);
    //public static DigitalInput elevatorSensor4 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_4);

    //public static DigitalInput cargoIntakeSensorOne = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_1);
    //public static DigitalInput cargoIntakeSensorTwo = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_2);
    //public static DigitalInput cargoIntakeSensorThree = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_3);

    public static AnalogSensor floorSensorOne = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_1, Drive.FLOOR_SENSOR_THRESHOLD);
    public static DigitalInput floorSensorTwo = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_2);
    public static AnalogSensor floorSensorThree = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_3, Drive.FLOOR_SENSOR_THRESHOLD);
    public static DigitalInput floorSensorFour = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT_4);
    public static AnalogSensor floorSensorFive = new AnalogSensor(RobotMap.FLOOR_SENSOR_PORT_5, Drive.FLOOR_SENSOR_THRESHOLD);

    public static DigitalInput forceSensorOne = new DigitalInput(RobotMap.FORCE_SENSOR_1);
    public static DigitalInput forceSensorTwo = new DigitalInput(RobotMap.FORCE_SENSOR_2);
    //public static DigitalInput forceSensorThree = new DigitalInput(RobotMap.FORCE_SENSOR_3);

}

