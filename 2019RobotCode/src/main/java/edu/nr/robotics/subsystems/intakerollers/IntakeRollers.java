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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollers extends NRSubsystem {

    private static IntakeRollers singleton;

    private VictorSPX intakeRollers;
    private PowerDistributionPanel pdp;

    public static Time  VOLTAGE_RAMP_RATE_INTAKE_ROLLERS = new Time(0.05, Time.Unit.SECOND);

    public static double INTAKE_PERCENT = 0; //find all
    public static double OUTTAKE_PERCENT = 0; 

    public static final int PEAK_CURRENT_INTAKE_ROLLERS = 80;
    public static final int PEAK_CURRENT_DURATION_INTAKE_ROLLERS = 250;
    public static final int CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS = 40;

    public static final int VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS = 12;

    public static final NeutralMode NEUTRAL_MODE_INTAKE_ROLLERS = NeutralMode.Brake;

    public static final double CURRENT_PEAK_INTAKE_ROLLERS = 0; //TODO: find

    public static final int DEFAULT_TIMEOUT = 0;
    
    public static Time SCORE_TIME = Time.ZERO; // find

    //setpoint of subsystem motor velocity
    public double Vel_Setpoint = 0; 

    private IntakeRollers() {

        if(EnabledSubsystems.INTAKE_ROLLERS_ENABLED){

            intakeRollers = CTRECreator.createMasterVictor(RobotMap.INTAKE_ROLLERS);
            pdp = new PowerDistributionPanel();

            intakeRollers.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
            intakeRollers.setInverted(true);
            
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

    public double getCurrent() {
        if(intakeRollers != null) {
            return pdp.getCurrent(RobotMap.INTAKE_ROLLERS_CURRENT);
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
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_BASIC_ENABLED) {
            SmartDashboard.putNumber("Intake Rollers Current: ", getCurrent());
        }
        if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
            Vel_Setpoint = SmartDashboard.getNumber("Intake Rollers Vel Percent: ", Vel_Setpoint);
        }
	}

	@Override
	public void disable() {
        setMotorPercent(0);
    }


}
