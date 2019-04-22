package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism;

public class EnabledSensors {
    
    public static boolean limelightEnabled = false;
    public static boolean lowGoalSensorEnabled = false;
    public static boolean cargoIntakeSensorEnabled = false;
    public static boolean floorSensorEnabled = false;

    //more sensors, way more sensors

    public static DigitalSensor elevatorSensor = new DigitalSensor(RobotMap.ELEVATOR_SENSOR_PORT, false);
    //public static DigitalInput elevatorSensor2 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_2);
    //public static DigitalInput elevatorSensor3 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_3);
    //public static DigitalInput elevatorSensor4 = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT_4);

    //public static DigitalInput cargoIntakeSensorOne = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_1);
    //public static DigitalInput cargoIntakeSensorTwo = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_2);
    //public static DigitalInput cargoIntakeSensorThree = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR_PORT_3);

    public static DigitalSensor floorSensorOne = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_1, true);
    public static DigitalSensor floorSensorTwo = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_2, true);
    public static DigitalSensor floorSensorThree = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_3, false);
    public static DigitalSensor floorSensorFour = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_4, true);
    public static DigitalSensor floorSensorFive = new DigitalSensor(RobotMap.FLOOR_SENSOR_PORT_5, true);

    public static DigitalSensor platformSensor = new DigitalSensor(RobotMap.PLATFORM_SENSOR, false);

    public static AnalogSensor hatchSensor1 = new AnalogSensor(RobotMap.HATCH_SENSOR_1, HatchMechanism.HATCH_SENSOR_THRESHOLD, false);
    public static AnalogSensor hatchSensor2 = new AnalogSensor(RobotMap.HATCH_SENSOR_2, HatchMechanism.HATCH_SENSOR_THRESHOLD, false);

    //public static DigitalSensor forceSensorOne = new DigitalSensor(RobotMap.FORCE_SENSOR_1);
    //public static DigitalSensor forceSensorTwo = new DigitalSensor(RobotMap.FORCE_SENSOR_2);
    //public static DigitalSensor forceSensorThree = new DigitalSensor(RobotMap.FORCE_SENSOR_3);

}

