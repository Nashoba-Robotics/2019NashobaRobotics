package edu.nr.robotics.subsystems.drive;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Angle;
import edu.nr.lib.NRMath;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.drive.DriveJoystickCommand;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.motorcontollers.*;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.subsystems.drive.CheesyDriveCalculationConstants;
import edu.nr.lib.motionprofiling.RampedDiagonalHTrajectory;


public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
    
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

					leftDriveFollow = CTRECreator.createFollowerVictor(RobotMap.LEFT_DRIVE_FOLLOW, leftDrive);
					rightDriveFollow = CTRECreator.createFollowerVictor(RobotMap.RIGHT_DRIVE_FOLLOW, rightDrive);
					hDriveFollow = CTRECreator.createFollowerVictor(RobotMap.H_DRIVE_FOLLOW, hDrive);

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

					leftDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
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
				return new Distance(hDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_H);
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
					return new Speed(hDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
			return Speed.ZERO;	
			}


		public double getRightCurrent() {
				if(rightDrive != null) {
					return rightDrive.getOutputCurrent();
				}
				return 0;
		}

		public double getLeftCurrent() {
			if (leftDrive != null) {
				return leftDrive.getOutputCurrent();
			}
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
				hDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_H_RIGHT * hMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_H_LEFT) * 1023.0)
				 / hMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);

				if(EnabledSubsystems.DRIVE_DUMB_ENABLED) {
						leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.div(MAX_SPEED_DRIVE));
						rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.div(MAX_SPEED_DRIVE));
						hDrive.set(hDrive.getControlMode(), hMotorSetpoint.div(MAX_SPEED_DRIVE_H));
				} else {
					leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
					rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
					if (Math.abs(hMotorSetpoint.div(MAX_SPEED_DRIVE_H)) < 0.05){
							hDrive.set(ControlMode.PercentOutput, 0);
				} else 
						hDrive.set(ControlMode.Velocity, hMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND));

			}

			}
		}


		public void setVoltageRamp(Time time) {
				leftDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				rightDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				leftDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				rightDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		}

		public void setVoltageRampH(Time time) {
				hDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				hDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
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

		public void enableMotionProfiler(Distance distX, Distance distY, double maxVelPercent, double maxAccelPercent) {
			double minVel;
			double minAccel;

			if(distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
				minVel = MAX_SPEED_DRIVE_H.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
			}  else if (distY.equals(Distance.ZERO) && !distX.equals(Distance.ZERO)) {
						minVel = MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
						minAccel = MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND);
			}	else if(!distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
					minVel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND),
					 (NRMath.hypot(distX, distY).div(distY)) * MAX_SPEED_DRIVE_H.mul(drivePercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND));
					minAccel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND,
					 Time.Unit.HUNDRED_MILLISECOND), (NRMath.hypot(distX, distY).div(distY)) * MAX_ACCEL_DRIVE_H.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND));
				} else {
					minVel = 0;
					minAccel = 0;
					System.out.println("No Distances Set");
				}
//fix 
				diagonalProfiler = new OneDimensionalMotionProfilerTwoMotor(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD);
				diagonalProfiler.setTrajectory(new RampedDiagonalHTrajectory(distX.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), distY.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H), minVel, minAccel));
				diagonalProfiler.enable();
	}

	public void disableProfiler() {
		diagonalProfiler.disable();
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
		}
	
	}

	public void smartDashboardInfo() {
		if(leftDrive != null && rightDrive != null){
			if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {

				SmartDashboard.putNumber("Drive Left Current", getLeftCurrent());
				SmartDashboard.putNumber("Drive Right Current", getRightCurrent());
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
			SmartDashboard.putNumber("Drive H Encoder Position", hDrive.getSelectedSensorPosition(PID_TYPE));
		
			leftDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Left P Value: ", P_LEFT), DEFAULT_TIMEOUT);
			leftDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Left I Value: ", I_LEFT), DEFAULT_TIMEOUT);
			leftDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Left D Value: ", D_LEFT), DEFAULT_TIMEOUT);

			rightDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), DEFAULT_TIMEOUT);
			rightDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), DEFAULT_TIMEOUT);
			rightDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), DEFAULT_TIMEOUT);

			hDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("H P Value: ", P_H_RIGHT), DEFAULT_TIMEOUT);
			hDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("H I Value: ", I_H_RIGHT), DEFAULT_TIMEOUT);
			hDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("H D Value: ", D_H_RIGHT), DEFAULT_TIMEOUT);

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