package edu.nr.robotics.subsystems.auxiliarydrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motorcontrollers.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AuxiliaryDrive extends NRSubsystem {

	private static AuxiliaryDrive singleton;

	private TalonSRX auxDrive;

	private PowerDistributionPanel pdp;

	public static final Speed MAX_SPEED_AUX_DRIVE = Speed.ZERO;
	public static final Acceleration MAX_ACCEL_AUX_DRIVE = Acceleration.ZERO;

	public static final double REAL_ENC_TICK_PER_INCH_AUX_DRIVE = 0;

	public static final double EFFECTIVE_ENC_TICK_PER_INCH_AUX_DRIVE = 0;

	public static final Distance WHEEL_DIAMETER = new Distance(0, Distance.Unit.INCH);
	public static final Distance WHEEL_DIAMETER_EFFECTIVE = new Distance(0, Distance.Unit.INCH);
	
	public static Time auxDriveRampRate = Time.ZERO;

	//Type of PID. 0 = primary. 1 = cascade
	public static final int PID_TYPE = 0;

	public static final int VEL_SLOT = 0;

	//No timeout for talon configuration functions
	public static final int DEFAULT_TIMEOUT = 0;

	public static final double P = 0;
	public static final double I = 0;
	public static final double D = 0;

	public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // find
	public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; //find this

	public static final int VOLTAGE_COMPENSATION_LEVEL = 12;	
	private static final int PEAK_AUX_DRIVE_CURRENT = 80;//amps
	private static final int PEAK_AUX_DRIVE_CURRENT_DURATION = 1000;//miliseconds, so one second
	private static final int CONTINUOUS_CURRENT_LIMIT = 40; //amps

	public static final double MIN_MOVE_VOLTAGE_PERCENT = 0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE = 0;

	public static final double CLIMB_HOLD_VOLTAGE = 0.01;

	private Speed motorSetpoint = Speed.ZERO;

	private PIDSourceType type = PIDSourceType.kRate;


    public AuxiliaryDrive() {
		if (EnabledSubsystems.AUX_DRIVE_ENABLED) {
			auxDrive = CTRECreator.createMasterTalon(RobotMap.AUX_DRIVE);
			pdp = new PowerDistributionPanel(RobotMap.PDP_ID);
			
			if(EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				auxDrive.set(ControlMode.PercentOutput,0);
			} else {
				auxDrive.set(ControlMode.Velocity, 0);
			}

			//auxDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);

			auxDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			auxDrive.config_kP(VEL_SLOT, P, DEFAULT_TIMEOUT);
			auxDrive.config_kI(VEL_SLOT, I, DEFAULT_TIMEOUT);
			auxDrive.config_kD(VEL_SLOT, D, DEFAULT_TIMEOUT);

			auxDrive.setNeutralMode(NEUTRAL_MODE);
			auxDrive.setInverted(false);
			auxDrive.setSensorPhase(false);

			auxDrive.enableVoltageCompensation(true);
			auxDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);

			auxDrive.enableCurrentLimit(true);
			auxDrive.configPeakCurrentLimit(PEAK_AUX_DRIVE_CURRENT, DEFAULT_TIMEOUT);
			auxDrive.configPeakCurrentDuration(PEAK_AUX_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
			auxDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);

			auxDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
			auxDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);

			auxDrive.configClosedloopRamp(auxDriveRampRate.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			auxDrive.configOpenloopRamp(auxDriveRampRate.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			
			auxDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);

			smartDashboardInit();
		}
	}

	public synchronized static void init() {
		if(singleton == null) {
			singleton = new AuxiliaryDrive();
			if(EnabledSubsystems.DRIVE_ENABLED)
			singleton.setJoystickCommand(new AuxiliaryDriveJoystickCommand());
		}
	}

	public static AuxiliaryDrive getInstance() {
		if(singleton == null)
			init();
		return singleton;
	}

	public Distance getPosition() {
		if(auxDrive != null){
			return new Distance(auxDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_AUX_DRIVE);
		} else {
			return Distance.ZERO;
		}
	}

	public Speed getVelocity() {
		if(auxDrive != null)
			return new Speed(auxDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_AUX_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}

	public double getCurrent() {
		if(auxDrive != null) {
			return auxDrive.getOutputCurrent();
		}
		return 0;
	}

	public void setMotorSpeedRaw(double percent) {
		auxDrive.set(ControlMode.PercentOutput, percent);
	}

	public void setMotorSpeedInPercent(double percent) {
		if (EnabledSubsystems.AUX_DRIVE_DUMB_ENABLED)
			setMotorSpeedRaw(percent);
		else
			setMotorSpeed(MAX_SPEED_AUX_DRIVE.mul(percent));	
	}

	public void setMotorSpeed(Speed speed) {
		if(auxDrive != null) {

			motorSetpoint = speed;

			auxDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE * motorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT) * 1023.0) 
			/ motorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);

			if(EnabledSubsystems.AUX_DRIVE_DUMB_ENABLED) {
				auxDrive.set(auxDrive.getControlMode(), motorSetpoint.div(MAX_SPEED_AUX_DRIVE));
			} else {
				auxDrive.set(auxDrive.getControlMode(), motorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));

			}

		}
	}

	public void setVoltageRampH(Time time) {
		auxDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		auxDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	}

	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	public PIDSourceType getPIDSourceType() {
		return type;
	}

	public double pidGet() {
		if(type == PIDSourceType.kRate) {
			return getInstance().getVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}
	
	public void pidWrite(double output) {
		setMotorSpeedInPercent(output);
	}

	public void smartDashboardInit() {
		if(EnabledSubsystems.AUX_DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Aux Drive P Value: ", P);
			SmartDashboard.putNumber("Aux Drive I Value: ", I);
			SmartDashboard.putNumber("Aux Drive D Value: ", D);

			SmartDashboard.putNumber("Aux Drive Ramp Rate: ", auxDriveRampRate.get(Time.Unit.SECOND));
			
		}
	}

	@Override
	public void smartDashboardInfo() {
		if(auxDrive != null){
			if(EnabledSubsystems.AUX_DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
				SmartDashboard.putNumber("Aux Drive Current", getCurrent());

				SmartDashboard.putNumber("Aux Drive Velocity", getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
			}
			if(EnabledSubsystems.AUX_DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
				SmartDashboard.putNumber("Aux Drive Percent", motorSetpoint.div(MAX_SPEED_AUX_DRIVE));
				
				SmartDashboard.putNumber("Aux Drive Position", getPosition().get(Distance.Unit.INCH));

				SmartDashboard.putNumber("Aux Drive Encoder Position", auxDrive.getSelectedSensorPosition(PID_TYPE));
			
				auxDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Aux Drive P Value: ", P), DEFAULT_TIMEOUT);

				auxDriveRampRate = new Time(SmartDashboard.getNumber("Drive Ramp Rate: ", auxDriveRampRate.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			}	
		}
	}

	@Override
	public void disable() {
		setMotorSpeedInPercent(0);
	}
}
