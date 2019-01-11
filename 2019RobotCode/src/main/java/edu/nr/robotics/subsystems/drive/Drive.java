package edu.nr.robotics.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
    //redo
    private static Drive singleton;

		private TalonSRX  leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, hDrive, hDriveFollow, pigeonTalon; 
		//these may change because of new talons


		//fix all of these
		public static final double REAL_ENC_TICK_PER_INNCH_DRIVE = 0;
		public static final double REAL_ENC_TICK_PER_INCH_H_DRIVE = 0;

		public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = 0;
		public static final double EFFECCTIVE_ENC_TICK_PER_INCH_H_DRIVE = 0;

		public static final Speed MAX_SPEED_DRIVE = new Speed();
		public static final Speed MAX_SPEED_DRIVE_H = new Speed();

		public static final Acceleration MAX_ACCEL_DRIVE = new Acceleration();
		public static final Acceleration MAX_ACCEL_DRIVE_H = new Acceleration();

		public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0;
		public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0;

		public static final double MIN_MOVE_VOLTAGE_PERCENT_H_LEFT = 0;
		public static final double MIN_MOVE_VOLTAGE_PERCENT_H_RIGHT = 0;

		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0;
		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;

		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H_LEFT = 0;
		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H_RIGHT = 0;

		public static Time DRIVE_RAMP_RATE = new Time();
		public static Time H_DRIVE_RAMP_RATE = new Time();

		public static double P_LEFT = 0;
		public static double I_LEFT = 0;
		public static double D_LEFT = 0;

		public static double P_RIGHT = 0;
		public static double I_RIGHT = 0;
		public static double D_RIGHT = 0;

		public static double P_H_LEFT = 0;
		public static double I_H_LEFT = 0;
		public static double D_H_LEFT = 0;

		public static double P_H_RIGHT = 0;
		public static double I_H_RIGHT = 0;
		public static double D_H_RIGHT = 0;

		public static double kVOneD = 1 / MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		public static double kAOneD = 0.0;
		public static double kPOneD = 0.0;
		public static double kIOneD = 0;
		public static double kDOneD = 0;
		public static double kP_thetaOneD = 0;

		public static double kVOneDH = 1 / MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
		public static double kAOneDH = 0;
		public static double kPOneDH = 0;
		public static double kIOneDH = 0;
		public static double kDOneDH = 0;

		public static final double PROFILE_DRIVE_PERCENT = 0;
		public static final double ACCEL_PERCENT = 0;

		public static double TURN_JOYSTICK_MULTIPLIER = 0;
		public static double MOVE_JOYSTICK_MULTIPLIER = 0;

		public static final double MAX_PROFILLE_TURN_PERCENT = 0;
		public static final double MIN_PROFILE_TURN_PERCENT = 0;

		public static final double DRIVE_TO_HATCH_PERCENT = 0;
		public static final double DRIVE_TO_CARGO_PERCENT = 0;

		public static final double SENSOR_STRAFE_PERCENT = 0;



	








}