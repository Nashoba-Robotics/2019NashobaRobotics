package edu.nr.robotics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static final int DRIVE_LEFT = 9;
	public static final int DRIVE_RIGHT = 6;
	public static final int DRIVE_RIGHT_FOLLOW = 2;
	public static final int DRIVE_LEFT_FOLLOW = 3;
	public static final int PIGEON_TALON = 1;

	public static final int FORCE_SENSOR_ONE_PORT = 0;
	public static final int FORCE_SENSOR_TWO_PORT = 1;
}
