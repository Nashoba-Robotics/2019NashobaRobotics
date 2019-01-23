package edu.nr.robotics.subsystems.drive;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.NRMath;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Waypoint;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.drive.DriveJoystickCommand;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.motorcontrollers.CTRECreator;
import edu.nr.lib.motorcontrollers.SparkMax;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.subsystems.drive.CheesyDriveCalculationConstants;
import edu.nr.lib.motionprofiling.RampedDiagonalHTrajectory;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;


public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
    
    private static Drive singleton;

		private TalonSRX  leftDrive, rightDrive, pigeonTalon; 
		private VictorSPX leftDriveFollow1, leftDriveFollow2, rightDriveFollow1, rightDriveFollow2;
		private CANSparkMax hDrive;
		private PowerDistributionPanel pdp;
		//these may change because of new talons


		//fix all of these
		public static final double REAL_ENC_TICK_PER_INNCH_DRIVE = 0;
		public static final double REAL_ENC_TICK_PER_INCH_H_DRIVE = 0;

		public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = 0;
		public static final double EFFECTIVE_ENC_REV_PER_INCH_H_DRIVE = 0;

		public static final Distance WHEEL_DIAMETER = new Distance(6, Distance.Unit.INCH);
		public static final Distance WHEEL_DIAMETER_EFFECTIVE = new Distance(6, Distance.Unit.INCH);
	
		public static final Distance WHEEL_BASE = Distance.ZERO;

		public static final Speed MAX_SPEED_DRIVE = Speed.ZERO;
		public static final Speed MAX_SPEED_DRIVE_H = Speed.ZERO;

		public static final Acceleration MAX_ACCEL_DRIVE = Acceleration.ZERO;
		public static final Acceleration MAX_ACCEL_DRIVE_H = Acceleration.ZERO;

		public static final Jerk MAX_JERK_DRIVE = Jerk.ZERO;

		public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0;
		public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0;

		public static final double MIN_MOVE_VOLTAGE_PERCENT_H_LEFT = 0;
		public static final double MIN_MOVE_VOLTAGE_PERCENT_H_RIGHT = 0;

		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0;
		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;

		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H_LEFT = 0;
		public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H_RIGHT = 0;

		public static Time DRIVE_RAMP_RATE = Time.ZERO;
		public static Time H_DRIVE_RAMP_RATE = Time.ZERO;

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

		public static double kVOneDH = 1 / MAX_SPEED_DRIVE.get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND);
		public static double kAOneDH = 0;
		public static double kPOneDH = 0;
		public static double kIOneDH = 0;
		public static double kDOneDH = 0;
		
		public static double kVTwoD = 1
		/ MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		public static double kATwoD = 0;
		public static double kPTwoD = 0;
		public static double kITwoD = 0;
		public static double kDTwoD = 0;
		public static double kP_thetaTwoD = 0;

		public static final double PROFILE_DRIVE_PERCENT = 0;
		public static final double ACCEL_PERCENT = 0;

		public static double TURN_JOYSTICK_MULTIPLIER = 0;
		public static double MOVE_JOYSTICK_MULTIPLIER = 0;

		public static final double MAX_PROFILE_TURN_PERCENT = 0;
		public static final double MIN_PROFILE_TURN_PERCENT = 0;

		public static final double DRIVE_TO_HATCH_PERCENT = 0;
		public static final double DRIVE_TO_CARGO_PERCENT = 0;

		public static final double SENSOR_STRAFE_PERCENT = 0;

		public static final Distance END_THRESHOLD = Distance.ZERO;
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
		public static Angle endAngle = new Angle(0, Angle.Unit.DEGREE); // set next two to SD...
		public static String profileName = "ProfileName";
		public static double drivePercent;
		public static double accelPercent;
		public static Angle angleToTurn;

		private OneDimensionalMotionProfilerTwoMotor diagonalProfiler;
		private TwoDimensionalMotionProfilerPathfinder twoDProfiler;
		private Waypoint[] points;

		public static enum DriveMode {
			arcadeDrive, tankDrive, cheesyDrive, fieldCentricDrive
		}

		private Drive() {
			if(EnabledSubsystems.DRIVE_ENABLED) {
				leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE);
				rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE);
				hDrive = SparkMax.createSpark(RobotMap.H_DRIVE, true);

				leftDriveFollow1 = CTRECreator.createFollowerVictor(RobotMap.LEFT_DRIVE_FOLLOW_1, leftDrive);
				leftDriveFollow2 = CTRECreator.createFollowerVictor(RobotMap.LEFT_DRIVE_FOLLOW_2, leftDrive);
				rightDriveFollow1 = CTRECreator.createFollowerVictor(RobotMap.RIGHT_DRIVE_FOLLOW_1, rightDrive);
				rightDriveFollow2 = CTRECreator.createFollowerVictor(RobotMap.RIGHT_DRIVE_FOLLOW_2, rightDrive);

				pigeonTalon = CTRECreator.createMasterTalon(RobotMap.PIGEON_TALON);


				if(EnabledSubsystems.DRIVE_DUMB_ENABLED) {
					leftDrive.set(ControlMode.PercentOutput,0);
					rightDrive.set(ControlMode.PercentOutput, 0); // .set sets sparkmax motor as a percent auomatically
					hDrive.set(0);
				} else {
					leftDrive.set(ControlMode.Velocity, 0);
					rightDrive.set(ControlMode.Velocity, 0);
					hDrive.getPIDController().setReference(0, ControlType.kVelocity, VEL_SLOT); // velocity set
				}

				leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

				leftDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
				leftDrive.config_kP(VEL_SLOT, P_LEFT, DEFAULT_TIMEOUT);
				leftDrive.config_kI(VEL_SLOT, I_LEFT, DEFAULT_TIMEOUT);
				leftDrive.config_kD(VEL_SLOT, D_LEFT, DEFAULT_TIMEOUT);

				leftDrive.setNeutralMode(NEUTRAL_MODE);
				leftDriveFollow1.setNeutralMode(NEUTRAL_MODE);
				leftDriveFollow2.setNeutralMode(NEUTRAL_MODE);
				leftDrive.setInverted(false);
				leftDriveFollow1.setInverted(false);
				leftDriveFollow2.setInverted(false);

				leftDrive.setSensorPhase(false);
				leftDriveFollow1.setSensorPhase(false);
				leftDriveFollow2.setSensorPhase(false);

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

				rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

				rightDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
				rightDrive.config_kP(VEL_SLOT, P_RIGHT, DEFAULT_TIMEOUT);
				rightDrive.config_kI(VEL_SLOT, I_RIGHT, DEFAULT_TIMEOUT);
				rightDrive.config_kD(VEL_SLOT, D_RIGHT, DEFAULT_TIMEOUT);

				rightDrive.setNeutralMode(NEUTRAL_MODE);
				rightDriveFollow1.setNeutralMode(NEUTRAL_MODE);
				rightDriveFollow2.setNeutralMode(NEUTRAL_MODE);
				
				rightDrive.setInverted(true);
				rightDriveFollow1.setInverted(true);
				rightDriveFollow2.setInverted(true);

				rightDrive.setSensorPhase(false);
				rightDriveFollow1.setSensorPhase(false);
				rightDriveFollow2.setSensorPhase(false);

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

				hDrive.getPIDController().setFF(0, VEL_SLOT);
				hDrive.getPIDController().setP(0, VEL_SLOT);
				hDrive.getPIDController().setI(0, VEL_SLOT);
				hDrive.getPIDController().setD(0, VEL_SLOT);

				hDrive.setIdleMode(IdleMode.kBrake);

				hDrive.setInverted(false);
				
				hDrive.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
				hDrive.setSecondaryCurrentLimit(PEAK_DRIVE_CURRENT);
				
				hDrive.setRampRate(H_DRIVE_RAMP_RATE.get(Time.Unit.SECOND));

				hDrive.getPIDController().setOutputRange(-1, 1);
				
			
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

		public Distance getRightPosition() {
			if(rightDrive != null){
				return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
			} else {
				return Distance.ZERO;
			}
		}

		public Distance getLeftPosition() {
			if(leftDrive != null){
				return new Distance(leftDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
			} else {
				return Distance.ZERO;
			}
		}

		public Distance getHPosition() {
			if(hDrive != null){
				return new Distance(hDrive.getEncoder().getPosition(), Unit.ENCODER_REV_H); // already integrated for h drive?
			} else {
				return Distance.ZERO;
			}
		}

		public Speed getLeftVelocity() {
			if(leftDrive != null)
					return new Speed(leftDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
			return Speed.ZERO;
		}

		public Speed getRightVelocity() {
			if(rightDrive != null)
					return new Speed(rightDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
			return Speed.ZERO;
		}

		public Speed getHVelocity() {
			if(hDrive != null)
					return new Speed(hDrive.get(), Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND);
			return Speed.ZERO;	
			}


		public double getRightCurrent() {
				if(rightDrive != null) {
					return rightDrive.getOutputCurrent();
				}
				return 0;
		}

		public double getRightFollow1Current() {
			if (rightDriveFollow1 != null)
				return pdp.getCurrent(RobotMap.RIGHT_DRIVE_FOLLOW_1_CURRENT);
			return 0;
		}

		public double getRightFollow2Current() {
			if (rightDriveFollow2 != null)
				return pdp.getCurrent(RobotMap.RIGHT_DRIVE_FOLLOW_2_CURRENT);
			return 0;
		}

		public double getLeftCurrent() {
			if (leftDrive != null) {
				return leftDrive.getOutputCurrent();
			}
			return 0;
		}

		public double getLeftFollow1Current() {
			if (leftDriveFollow1 != null) 
				return pdp.getCurrent(RobotMap.LEFT_DRIVE_FOLLOW_1_CURRENT);
			return 0;
		}

		public double getLeftFollow2Current() {
			if (leftDriveFollow2 != null) 
				return pdp.getCurrent(RobotMap.LEFT_DRIVE_FOLLOW_2_CURRENT);
			return 0;
		}

		public double getHCurrent() {
			if(hDrive != null) {
				return hDrive.getOutputCurrent();
			}
			return 0;
		}

		public void setMotorSpeedInPercent(double left, double right, double strafe) {

			if (OI.driveMode == DriveMode.fieldCentricDrive) {
				setMotorSpeed(MAX_SPEED_DRIVE_H.mul(left), MAX_SPEED_DRIVE_H.mul(right), MAX_SPEED_DRIVE_H.mul(strafe));
	
			} else {
				setMotorSpeed(MAX_SPEED_DRIVE.mul(left), MAX_SPEED_DRIVE.mul(right), MAX_SPEED_DRIVE_H.mul(strafe));	
	
			}
		}

		public void setMotorSpeed(Speed left, Speed right, Speed strafe) {
			if(leftDrive != null && rightDrive != null && hDrive != null) {

				leftMotorSetpoint = left;
				rightMotorSetpoint = right;
				hMotorSetpoint = strafe;

				//System.out.println(hMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));

				leftDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) 
				/ leftMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);
				rightDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) 
				/ rightMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);
				hDrive.getPIDController().setFF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_H_RIGHT * hMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_H_LEFT) * 1023.0)
				 / hMotorSetpoint.abs().get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND) ,VEL_SLOT); //fix later  , 1023 will change to match new native encoder unit ratios

				if(EnabledSubsystems.DRIVE_DUMB_ENABLED) {
						leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.div(MAX_SPEED_DRIVE));
						rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.div(MAX_SPEED_DRIVE));
						hDrive.set(hMotorSetpoint.div(MAX_SPEED_DRIVE_H)); // set in speed as a percent
				} else {
					leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
					rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
					if (Math.abs(hMotorSetpoint.div(MAX_SPEED_DRIVE_H)) < 0.05){
							hDrive.getPIDController().setReference(0, ControlType.kVelocity); //stop if velocity is really low
				} else 
						hDrive.getPIDController().setReference(hMotorSetpoint.get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND), ControlType.kVelocity);

			}

			}
		}


		public void setVoltageRamp(Time time) {
				leftDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				rightDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				leftDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				rightDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		}
//will it work?
		public void setVoltageRampH(Time time) {
				hDrive.setRampRate(time.get(Time.Unit.SECOND));
		}

		public void tankDrive(double left, double right, double strafe) {
			setMotorSpeedInPercent(left, right, strafe);
		}

		public void arcadeDrive(double move, double turn, double strafeRaw) {
			double [] motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);

			double strafe = NRMath.limit(strafeRaw);
			tankDrive(motorPercents[0], motorPercents[1], strafe);
		}

		public void cheesyDrive(double move, double turn, double strafe) {
			double [] cheesyMotorPercents = new double[2];
			cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, oldTurn, false);

			oldTurn = turn;

			tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1], strafe);

		}

		public TalonSRX getPigeonTalon() {
			return pigeonTalon;
		}

		public void setPIDSourceType(PIDSourceType pidSource) {
			type = pidSource;
		}


		public PIDSourceType getPIDSourceType() {
			return type;
		}

		public double pidGetRight() {
			if(type == PIDSourceType.kRate) {
				return getInstance().getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
			} else {
				return getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
			}
		}

		public double pidGetLeft() {
			if(type == PIDSourceType.kRate) {
				return getInstance().getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
			} else {
				return getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
			}
		}

		public void pidWrite(double outputLeft, double outputRight) {
			setMotorSpeedInPercent(outputLeft, outputRight, 0);
		}

		public void enableTwoDMotionProfiler(Distance distX, Distance distY, Angle endAngle, double maxVelPercent,
			double maxAccelPercent, String profileName) {
			File profileFile = new File("home/lvuser/" + profileName + ".csv");
			
			twoDProfiler = new TwoDimensionalMotionProfilerPathfinder(this, this, kVTwoD, kATwoD, kPTwoD, kITwoD, kDTwoD,
					kP_thetaTwoD,
					MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
							Time.Unit.HUNDRED_MILLISECOND),
					MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
							Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					MAX_JERK_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND,
							Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					(int) (Math.PI * WHEEL_DIAMETER_EFFECTIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)),
					WHEEL_DIAMETER.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE),
					WHEEL_BASE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), false, profileFile);

			System.out.println(profileFile.getName());
			
			if (!profileFile.exists()) {
				System.out.println("distX: " + distX.get(Distance.Unit.FOOT) + "	distY: "
						+ distY.get(Distance.Unit.FOOT) + "	end Angle: " + endAngle.get(Angle.Unit.DEGREE));
				points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(1, 0, 0),
						new Waypoint(distX.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE),
								distY.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), endAngle.get(Angle.Unit.RADIAN)) };
			}
				twoDProfiler.setTrajectory(points);
			
			twoDProfiler.enable();
			
		}

		public void enableMotionProfiler(Distance distX, Distance distY, double maxVelPercent, double maxAccelPercent) {
			double minVel;
			double minAccel = 0;

			if(distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
				minVel = MAX_SPEED_DRIVE_H.mul(maxVelPercent).get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND);
			}  else if (distY.equals(Distance.ZERO) && !distX.equals(Distance.ZERO)) {
						minVel = MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
						minAccel = MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND);
			}	else if(!distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
					minVel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND),
					 (NRMath.hypot(distX, distY).div(distY)) * MAX_SPEED_DRIVE_H.mul(drivePercent).get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND));
					minAccel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND,
					 Time.Unit.HUNDRED_MILLISECOND), (NRMath.hypot(distX, distY).div(distY)) * MAX_ACCEL_DRIVE_H.mul(maxAccelPercent).get(Distance.Unit.ENCODER_REV_H, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND));
				} else {
					minVel = 0;
					minAccel = 0;
					System.out.println("No Distances Set");
				}
//fix 
				diagonalProfiler = new OneDimensionalMotionProfilerTwoMotor(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD);
				diagonalProfiler.setTrajectory(new RampedDiagonalHTrajectory(distX.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), distY.get(Distance.Unit.ENCODER_REV_H), minVel, minAccel));
				diagonalProfiler.enable();
	}

	public void disableProfiler() {
		diagonalProfiler.disable();
		twoDProfiler.disable();
	}



	private void smartDashboardInit() {
		if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
				SmartDashboard.putData(new ResetGyroCommand());
		}
	 if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {

		SmartDashboard.putNumber("Left P Value: ", P_LEFT);
		SmartDashboard.putNumber("Left I Value: ", I_LEFT);
		SmartDashboard.putNumber("Left D Value: ", D_LEFT);

		SmartDashboard.putNumber("Right P Value: ", P_RIGHT);
		SmartDashboard.putNumber("Right I Value: ", I_RIGHT);
		SmartDashboard.putNumber("Right D Value: ", D_RIGHT);

		SmartDashboard.putNumber("H P Value: ", P_H_RIGHT);
		SmartDashboard.putNumber("H I Value: ", I_H_RIGHT);
		SmartDashboard.putNumber("H D Value: ", D_H_RIGHT);

		SmartDashboard.putNumber("kVOneD Value: ", kVOneD);
		SmartDashboard.putNumber("kAOneD Value: ", kAOneD);
		SmartDashboard.putNumber("kPOneD Value: ", kPOneD);
		SmartDashboard.putNumber("kIOneD Value: ", kIOneD);
		SmartDashboard.putNumber("kDOneD Value: ", kDOneD);
		SmartDashboard.putNumber("kP_thetaOneD Value: ", kP_thetaOneD);

		SmartDashboard.putNumber("kVOneDH Value: ", kVOneDH);
		SmartDashboard.putNumber("kAOneDH Value: ", kAOneDH);
		SmartDashboard.putNumber("kPOneDH Value: ", kPOneDH);
		SmartDashboard.putNumber("kIOneDH Value: ", kIOneDH);
		SmartDashboard.putNumber("kDOneDH Value: ", kDOneDH);

		SmartDashboard.putNumber("Drive Ramp Rate: ", DRIVE_RAMP_RATE.get(Time.Unit.SECOND));
		SmartDashboard.putNumber("H Drive Ramp Rate: ", H_DRIVE_RAMP_RATE.get(Time.Unit.SECOND));
		
		SmartDashboard.putNumber("X Profile Feet: ", 0);
		SmartDashboard.putNumber("Y Profile Feet: ", 0);
		SmartDashboard.putNumber("Drive Percent: ", PROFILE_DRIVE_PERCENT);
		SmartDashboard.putNumber("Drive Accel Percent: ", ACCEL_PERCENT);
		SmartDashboard.putNumber("Angle To Turn: ", 0);

		SmartDashboard.putNumber("Profile End Angle: ", endAngle.get(Angle.Unit.DEGREE));
		SmartDashboard.putString("Profile Name: ", profileName);
	}
	
	}

	public void smartDashboardInfo() {
		if(leftDrive != null && rightDrive != null){
			if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {

				SmartDashboard.putNumberArray("Drive Left Current", new double[] {getLeftCurrent(), getLeftFollow1Current(), getLeftFollow2Current()});
				SmartDashboard.putNumberArray("Drive Right Current", new double [] {getRightCurrent(), getRightFollow1Current(), getRightFollow2Current()});
				SmartDashboard.putNumber("Drive H Current", getHCurrent());

				SmartDashboard.putNumber("Gyro Yaw", (-Pigeon.getPigeon(getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE))% 360);

				SmartDashboard.putNumberArray("Drive Left Velocity", new double[] {getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
				SmartDashboard.putNumberArray("Drive Right Velocity", new double[] {getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
				SmartDashboard.putNumberArray("Drive H Velocity", new double[] {getHVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), hMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});

			}
		if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(MAX_SPEED_DRIVE));
			SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(MAX_SPEED_DRIVE));
			SmartDashboard.putNumber("Drive H Percent", hMotorSetpoint.div(MAX_SPEED_DRIVE_H));

			//SmartDashboard.putData(this); what why BEN WHY???

			SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.INCH));
			SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.INCH));
			SmartDashboard.putNumber("Drive H Position", getHPosition().get(Distance.Unit.INCH));

			SmartDashboard.putNumber("Drive Left Encoder Position", leftDrive.getSelectedSensorPosition(PID_TYPE));
			SmartDashboard.putNumber("Drive Right Encoder Position", rightDrive.getSelectedSensorPosition(PID_TYPE));
			SmartDashboard.putNumber("Drive H Encoder Position", hDrive.getEncoder().getPosition());
		
			leftDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Left P Value: ", P_LEFT), DEFAULT_TIMEOUT);
			leftDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Left I Value: ", I_LEFT), DEFAULT_TIMEOUT);
			leftDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Left D Value: ", D_LEFT), DEFAULT_TIMEOUT);

			rightDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), DEFAULT_TIMEOUT);
			rightDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), DEFAULT_TIMEOUT);
			rightDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), DEFAULT_TIMEOUT);

			hDrive.getPIDController().setP(SmartDashboard.getNumber("H P Value: ", P_H_RIGHT), VEL_SLOT);
			hDrive.getPIDController().setI(SmartDashboard.getNumber("H I Value: ", I_H_RIGHT), VEL_SLOT);
			hDrive.getPIDController().setD(SmartDashboard.getNumber("H D Value: ", D_H_RIGHT), VEL_SLOT);

			P_LEFT = SmartDashboard.getNumber("Left P Value: ", P_LEFT);
			I_LEFT = SmartDashboard.getNumber("Left I Value: ", I_LEFT);
			D_LEFT = SmartDashboard.getNumber("Left D Value: ", D_LEFT);

			P_RIGHT = SmartDashboard.getNumber("Right P Value: ", P_RIGHT);
			I_RIGHT = SmartDashboard.getNumber("Right I Value: ", I_RIGHT);
			D_RIGHT = SmartDashboard.getNumber("Right D Value: ", D_RIGHT);

			P_H_RIGHT = SmartDashboard.getNumber("H P Value: ", P_H_RIGHT);
			I_H_RIGHT = SmartDashboard.getNumber("H I Value: ", I_H_RIGHT);
			D_H_RIGHT = SmartDashboard.getNumber("H D Value: ", D_H_RIGHT);


			kVOneD = SmartDashboard.getNumber("kVOneD Value: ", kVOneD);
			kAOneD = SmartDashboard.getNumber("kAOneD Value: ", kAOneD);
			kPOneD = SmartDashboard.getNumber("kPOneD Value: ", kPOneD);
			kIOneD = SmartDashboard.getNumber("kIOneD Value: ", kIOneD);
			kDOneD = SmartDashboard.getNumber("kDOneD Value: ", kDOneD);
			kP_thetaOneD = SmartDashboard.getNumber("kP_thetaOneD Value: ", kP_thetaOneD);
				
			kVOneDH = SmartDashboard.getNumber("kVOneDH Value: ", kVOneDH);
			kAOneDH = SmartDashboard.getNumber("kAOneDH Value: ", kAOneDH);
			kPOneDH = SmartDashboard.getNumber("kPOneDH Value: ", kPOneDH);
			kIOneDH = SmartDashboard.getNumber("kIOneDH Value: ", kIOneDH);
			kDOneDH = SmartDashboard.getNumber("kDOneDH Value: ", kDOneDH);

			DRIVE_RAMP_RATE = new Time(SmartDashboard.getNumber("Drive Ramp Rate: ", DRIVE_RAMP_RATE.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			H_DRIVE_RAMP_RATE = new Time(SmartDashboard.getNumber("H Drive Ramp Rate: ", H_DRIVE_RAMP_RATE.get(Time.Unit.SECOND)), Time.Unit.SECOND);

			xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet: ", 0), Distance.Unit.FOOT);
			yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet: ", 0), Distance.Unit.FOOT);
			drivePercent = SmartDashboard.getNumber("Drive Percent: ", 0);
			accelPercent = SmartDashboard.getNumber("Drive Accel Percent: ", 0);
			angleToTurn = new Angle(SmartDashboard.getNumber("Angle To Turn: ", 0), Angle.Unit.DEGREE);

			endAngle = new Angle(SmartDashboard.getNumber("Profile End Angle: ", endAngle.get(Angle.Unit.DEGREE)), Angle.Unit.DEGREE);
			profileName = SmartDashboard.getString("Profile Name: ", profileName);
		}	

		}
	}

	public void periodic() {
		if (OI.getInstance().isKidModeOn()) {
			MOVE_JOYSTICK_MULTIPLIER = 0.6;
			if (!sniperModeEnabled) {
				TURN_JOYSTICK_MULTIPLIER = 0.6;	
			}
		} else {
			MOVE_JOYSTICK_MULTIPLIER = 1;
				if (!sniperModeEnabled) {
					TURN_JOYSTICK_MULTIPLIER = 1;	
				}		

		}

	}

	public void disable() {
		setMotorSpeedInPercent(0,0,0);
	}

	public void startDumbDrive() {
		if(leftDrive != null && rightDrive != null && hDrive != null) {
			EnabledSubsystems.DRIVE_DUMB_ENABLED = true;
		}
	}

	public void endDumbDrive() {
		if(leftDrive != null && rightDrive != null && hDrive != null) {
			EnabledSubsystems.DRIVE_DUMB_ENABLED = false;
		}
	}

}