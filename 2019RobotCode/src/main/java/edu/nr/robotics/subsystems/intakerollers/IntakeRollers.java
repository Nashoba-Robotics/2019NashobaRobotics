package edu.nr.robotics.subsystems.intakerollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motorcontrollers.CTRECreator;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollers extends NRSubsystem {

    private static IntakeRollers singleton;

    private TalonSRX intakeRollers;
    private PowerDistributionPanel pdp;

    private Solenoid deployRollers;

    public static Time  VOLTAGE_RAMP_RATE_INTAKE_ROLLERS = new Time(0.05, Time.Unit.SECOND);

    public static double INTAKE_PERCENT = 1; //find all
    public static double OUTTAKE_PERCENT = -0.85; 
    public static double HOLD_PERCENT = 0.2;

    public static final int PEAK_CURRENT_INTAKE_ROLLERS = 80;
    public static final int PEAK_CURRENT_DURATION_INTAKE_ROLLERS = 250;
    public static final int CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS = 40;

    public static final int VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS = 12;

    public static final NeutralMode NEUTRAL_MODE_INTAKE_ROLLERS = NeutralMode.Brake;

    public static final double CURRENT_PEAK_INTAKE_ROLLERS = 40; //TODO: find

    public static final int DEFAULT_TIMEOUT = 0;
    
    public static Time SCORE_TIME = new Time(2, Time.Unit.SECOND); // find

    public static Time ACTUATION_TIME = new Time(0.5, Time.Unit.SECOND);

    //setpoint of subsystem motor velocity
    public double Vel_Setpoint = 0; 

	public enum State {
		DEPLOYED, RETRACTED;
		
		private static boolean DEPLOYED_VALUE = true;
		private static boolean RETRACTED_VALUE = false;
		
		private static State get(boolean val) {
			if (val == State.DEPLOYED_VALUE) {
				return State.DEPLOYED;
			} else {
				return State.RETRACTED;
			}
		}
    }
    
	public State currentDeployState() {
		if(deployRollers != null) {
			return State.get(deployRollers.get());
		} else {
			return State.DEPLOYED; //TODO: Should be State.RETRACTED, is deployed for testing
		}
    }

    private IntakeRollers() {

        if(EnabledSubsystems.INTAKE_ROLLERS_ENABLED) {

            intakeRollers = CTRECreator.createMasterTalon(RobotMap.INTAKE_ROLLERS);
            pdp = new PowerDistributionPanel(RobotMap.PDP_ID);

            deployRollers = new Solenoid(RobotMap.PCM_ID, RobotMap.INTAKE_ROLLERS_PCM_PORT);

            intakeRollers.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
            intakeRollers.setInverted(true);

            intakeRollers.enableCurrentLimit(true);
			intakeRollers.configPeakCurrentLimit(PEAK_CURRENT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollers.configPeakCurrentDuration(PEAK_CURRENT_DURATION_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollers.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
            
            intakeRollers.enableVoltageCompensation(true);
            intakeRollers.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);         

			intakeRollers.configClosedloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
            intakeRollers.configOpenloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);

        }

        smartDashboardInit();
    }

    public static IntakeRollers getInstance() {
        if(singleton == null) {
            init();
        }
        return singleton;
    }

    public synchronized static void init() {
        if(singleton == null) {
            singleton = new IntakeRollers();
        }
    }

	void deployIntakeRollers() {
		if (deployRollers != null) {
			deployRollers.set(State.DEPLOYED_VALUE);
		}
	}

	void retractIntakeRollers() {
		if ((deployRollers != null) /*&& !(new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue())*/) {
			deployRollers.set(State.RETRACTED_VALUE);
		}
    }

    public double getCurrent() {
        if(intakeRollers != null) {
            return intakeRollers.getOutputCurrent();
        }
        return 0;
    }

    public void setMotorPercent(double percent) {
        if(intakeRollers != null) {
            Vel_Setpoint = percent;

            intakeRollers.set(ControlMode.PercentOutput, percent);

        }
    }

    public void smartDashboardInit() {
        if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putNumber("Intake Rollers Vel Percent: ", Vel_Setpoint);

        }
    }

	@Override
	public void smartDashboardInfo() {
        SmartDashboard.putBoolean("Intake Rollers Running: ", (intakeRollers.getMotorOutputPercent() == INTAKE_PERCENT));

		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_BASIC_ENABLED) {
            SmartDashboard.putString("Intake Rollers Deploy Position: ", currentDeployState().toString());
            SmartDashboard.putNumber("Intake Rollers Current: ", getCurrent());
        }
        if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
            Vel_Setpoint = SmartDashboard.getNumber("Intake Rollers Vel Percent: ", Vel_Setpoint);
        }
    }
    
    
	public boolean isIntakeRollersDeployed() {
		return currentDeployState() == State.DEPLOYED;
    }

	@Override
	public void disable() {
        setMotorPercent(0);
    }

    public boolean isRunning() {
        return intakeRollers.getMotorOutputPercent() != 0;
    }

    //public boolean hasCargo() {
    //    return !(new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue());
    //}


}
