package edu.nr.robotics.subsystems.drive;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Angle;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.RobotMap;
import edu.nr.lib.motorcontollers.*;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.subsystems.drive.CheesyDriveCalculationConstants;



public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
    //redo
    private static Drive singleton;

		private TalonSRX  leftDrive, rightDrive, hDrive, pigeonTalon; 
		private VictorSPX leftDriveFollow, rightDriveFollow, hDriveFollow; 
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

		public static final Distance END_THRESHOLD = new Distance();
		public static final Speed PROFILE_END_TURN_SPEED_THRESHOLD = MAX_SPEED_DRIVE.mul(MIN_PROFILE_TURN_PERCENT + 0.01);
		public static final Speed PROFILE_END_SPEED_THRESHOLD = MAX_SPEED_DRIVE.mul(0.10);

		public static final Angle DRIVE_ANGLE_THRESHOLD = new Angle(2, Angle.Unit.DEGREE); // change?
		public static final Angle DRIVE_STOP_ANGLE = new Angle(55, Angle.Unit.DEGREE); //find angle that robot stops at when turning goes from 1 to 0

		private static final int PEAK_DRIVE_CURRENT = 80;//amps
		private static final int PEAK_DRIVE_CURRENT_DURATION = 1000;//miliseconds, so one second
		private static final int CONTINUOUS_CURRENT_LIMIT = 40; //amps

		public static final double SWITCH_CURRENT_LIMIT = 70;
		//When Driving into an object, the current when the driving stops

		public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
		public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; //find this

		public static final int VOLTAGE_COMPENSATION_LEVEL = 12;

		public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

		//Type of PID. 0 = primary. 1 = cascade
		public static final int PID_TYPE = 0;

		public static final int VEL_SLOT = 0;

		//No timeout for talon configuration functions
		public static final int DEFAULT_TIMEOUT = 0;

		public static boolean sniperModeEnabled = false;

		//tracking drive motor setpoints

		private Speed leftMotorSetpoint = Speed.ZERO;
		private Speed rightMotorSetpoint = Speed.ZERO;
		private Speed hMotorSetpoint = Speed.ZERO;
		private double oldTurn;

		private PIDSourceType type = PIDSourceType.kRate;

		public static Distance xProfile;
		public static Distance yProfile;
		public static double drivePercent;
		public static double accelPercent;
		public static Angle angleToTurn;

		private OneDimensionalMotionProfilerTwoMotor diagonalProfiler;

		public static enum DriveMode {
			arcadeDrive, tankDrive, cheesyDrive, fieldCentricDrive
		}

		private Drive() {
			if(EnabledSubsystems.DRIVE_ENABLED) {
					leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE);
					rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE);
					hDrive = CTRECreator.createMasterTalon(RobotMap.H_DRIVE);

					leftDriveFollow = CTRECreator.createFollowerVictor(RobotMap.LEFT_DRIVE_FOLLOW, leftDrive.getDeviceID());
					rightDriveFollow = CTRECreator.createFollowerVictor(RobotMap.RIGHT_DRIVE_FOLLOW, rightDrive.getDeviceID());
					hDriveFollow = CTRECreator.createFollowerVictor(RobotMap.H_DRIVE_FOLLOW, hDrive.getDeviceID());

					pigeonTalon = CTRECreator.createMasterTalon(RobotMap.PIGEON_TALON);


					if(EnabledSubsystems.DRIVE_DUMB_ENABLED) {
						leftDrive.set(ControlMode.PercentOutput,0);
						rightDrive.set(ControlMode.PercentOutput, 0);
						hDrive.set(ControlMode.PercentOutput, 0);
					} else {
						leftDrive.set(ControlMode.Velocity, 0);
						rightDrive.set(ControlMode.Velocity, 0);
						hDrive.set(ControlMode.Velocity, 0);
					}

					leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

					leftDrive.config_KF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
					leftDrive.config_kP(VEL_SLOT, P_LEFT, DEFAULT_TIMEOUT);
					leftDrive.config_kI(VEL_SLOT, I_LEFT, DEFAULT_TIMEOUT);
					leftDrive.config_kD(VEL_SLOT, D_LEFT, DEFAULT_TIMEOUT);

					leftDrive.setNeutralMode(NEUTRAL_MODE);
				leftDrive.setInverted(false);

				leftDrive.setSensorPhase(false);
				leftDriveFollow.setSensorPhase(false);

				leftDrive.enableVoltageCompensation(true);
				leftDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);

				leftDrive.enableCurrentLimit(true);
				leftDrive.configPeakCurrentLimit(PEAK_DRIVE_CURRENT, DEFAULT_TIMEOUT);
				leftDrive.configPeakCurrentDuration(PEAK_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
				leftDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);

				leftDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
				leftDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);

				leftDrive.configClosedloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				leftDrive.configOpenloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				
				leftDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);

				leftDriveFollow.setNeutralMode(NEUTRAL_MODE);

				

			rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

			rightDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			rightDrive.config_kP(VEL_SLOT, P_RIGHT, DEFAULT_TIMEOUT);
			rightDrive.config_kI(VEL_SLOT, I_RIGHT, DEFAULT_TIMEOUT);
			rightDrive.config_kD(VEL_SLOT, D_RIGHT, DEFAULT_TIMEOUT);

			rightDrive.setNeutralMode(NEUTRAL_MODE);			
			rightDrive.setInverted(true);
			rightDriveFollow.setInverted(true);

			rightDrive.setSensorPhase(false);
			rightDriveFollow.setSensorPhase(false);

			rightDrive.enableVoltageCompensation(true);
			rightDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);

			rightDrive.enableCurrentLimit(true);
			rightDrive.configPeakCurrentLimit(PEAK_DRIVE_CURRENT, DEFAULT_TIMEOUT);
			rightDrive.configPeakCurrentDuration(PEAK_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
			rightDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);

			rightDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
			rightDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);

			rightDrive.configClosedloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			rightDrive.configOpenloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);

			rightDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);

			rightDriveFollow.setNeutralMode(NEUTRAL_MODE);


			hDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

			hDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			hDrive.config_kP(VEL_SLOT, P_LEFT, DEFAULT_TIMEOUT);
			hDrive.config_kI(VEL_SLOT, I_LEFT, DEFAULT_TIMEOUT);
			hDrive.config_kD(VEL_SLOT, D_LEFT, DEFAULT_TIMEOUT);

			hDrive.setNeutralMode(NEUTRAL_MODE);

			hDrive.setInverted(false);
			hDriveFollow.setInverted(false);

			hDrive.setSensorPhase(true);
			
			hDrive.enableVoltageCompensation(true);
			hDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);
			
			hDrive.enableCurrentLimit(true);
			hDrive.configPeakCurrentLimit(PEAK_DRIVE_CURRENT, DEFAULT_TIMEOUT);
			hDrive.configPeakCurrentDuration(PEAK_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
			hDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);

			hDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
			hDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);
			
			hDrive.configClosedloopRamp(H_DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			hDrive.configOpenloopRamp(H_DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			
			hDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);
					
			hDriveFollow.setNeutralMode(NEUTRAL_MODE);
		
			smartDashboardInit();

			CheesyDriveCalculationConstants.createDriveTypeCalculations();

			LimelightNetworkTable.getInstance();
				}
		}

		public synchronized static void init() {
			if(singleton == null) {
				singleton = new Drive();
				if(EnabledSubsystems.DRIVE_ENABLED)
				singleton.setJoystickCommand(new DriveJoystickCommand());
			}
		}

		public static Drive getInstance() {
			if(singleton == null)
				init();
			return singleton;
		}

		
}