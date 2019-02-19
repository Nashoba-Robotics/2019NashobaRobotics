
package edu.nr.robotics.subsystems.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motorcontrollers.SparkMax;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Time.Unit;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift extends NRSubsystem {

    private static Lift singleton;

    private CANSparkMax lift;

    public static final double INCH_PER_REVOLUTION_LIFT = 1 / (4.0928);

    public static final Speed MAX_SPEED_LIFT = Speed.ZERO;

    public static final Acceleration MAX_ACCEL_LIFT = Acceleration.ZERO;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT = 0;
    
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT = 0;
    
    public static Time VOLTAGE_RAMP_RATE_LIFT = Time.ZERO;
    public static final int VOLTAGE_COMPENSATION_LEVEL = 12;

    public static double F_POS_LIFT = 0; //Leave F at 0
    public static double P_POS_LIFT = 0;
    public static double I_POS_LIFT = 0;
    public static double D_POS_LIFT = 0;

    public static double F_VEL_LIFT = 0;
    public static double P_VEL_LIFT = 0;
    public static double I_VEL_LIFT = 0;
    public static double D_VEL_LIFT = 0;

    public static double P_Angle = 0;

    public static final int PEAK_CURRENT_LIFT = 60;
    //public static final int PEAK_CURRENT_DURATION_LIFT = 250;
    public static final int CONTINUOUS_CURRENT_LIMIT_LIFT = 40;

    public static double profilePercent = 0.9;

    public static final Distance PROFILE_END_THRESHOLD_LIFT = new Distance(1, Distance.Unit.INCH);
    public static final Speed PROFILE_STOP_SPEED_THRESHOLD = new Speed(0.1, Distance.Unit.INCH, Time.Unit.SECOND);

    public static final int VELOCITY_MEASUREMENT_PERIOD_LIFT = 10; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_LIFT = 32; //find this

    public static final IdleMode IDLE_MODE_LIFT = IdleMode.kBrake;

    //Type of PID. 0 = primary. 1 = cascade
    public static final int PID_TYPE = 0;
    
    public static final double HOLD_VOLTAGE = 0;

    //No timeout for spark configuration functions
    public static final int DEFAULT_TIMEOUT = 0;
    
    public static final int VEL_SLOT = 0;
    public static final int POS_SLOT = 1;

    public static final Distance TOP_POSITION = Distance.ZERO;
    public static final Distance LEVEL1_POS = Distance.ZERO;
    public static final Distance LEVEL2_POS = Distance.ZERO;
    public static final Distance BOTTOM_HEIGHT = Distance.ZERO;

    //tracking drive motor setpoints
    private Speed velSetpoint = Speed.ZERO;
    private Distance posSetpoint = Distance.ZERO;

    public static Distance setPos = Distance.ZERO;
    public static Distance deltaPos = Distance.ZERO;

    public static boolean deployed = false;

    private Lift() {
        if (EnabledSubsystems.LIFT_ENABLED) {
            lift = SparkMax.createSpark(RobotMap.LIFT, true);

            lift.getPIDController().setFF(F_VEL_LIFT, VEL_SLOT);
            lift.getPIDController().setP(P_VEL_LIFT, VEL_SLOT);
            lift.getPIDController().setI(I_VEL_LIFT, VEL_SLOT);
            lift.getPIDController().setD(D_VEL_LIFT, VEL_SLOT);

            lift.getPIDController().setFF(F_POS_LIFT, POS_SLOT);
            lift.getPIDController().setP(P_POS_LIFT, POS_SLOT);
            lift.getPIDController().setI(I_POS_LIFT, POS_SLOT);
            lift.getPIDController().setD(D_POS_LIFT, POS_SLOT);

            lift.setIdleMode(IDLE_MODE_LIFT);

            lift.setInverted(true);

            lift.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT_LIFT);
            lift.setSecondaryCurrentLimit(PEAK_CURRENT_LIFT);

            lift.enableVoltageCompensation(VOLTAGE_COMPENSATION_LEVEL);

            lift.setClosedLoopRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));
            lift.setOpenLoopRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));

            lift.getPIDController().setOutputRange(-1, 1, VEL_SLOT);
            lift.getPIDController().setOutputRange(-1, 1, POS_SLOT);

            lift.setEncPosition(0);

            smartDashboardInit();

        }

    }

    public static Lift getInstance() {
        if(singleton == null) {
            init();
        }
        return singleton;
    }

    public synchronized static void init() {
        if(singleton == null) {
            singleton = new Lift();
            if(EnabledSubsystems.LIFT_ENABLED)
            singleton.setJoystickCommand(new LiftJoystickCommand());
        }
    }

    public Distance getPosition() {
        if(lift != null){
            return new Distance(lift.getEncoder().getPosition(), Distance.Unit.ENCODER_REV_LIFT);
        } else {
            return Distance.ZERO;
        }
    }

    public Speed getVelocity() {
        if(lift != null){
            return new Speed(lift.getEncoder().getVelocity(), Distance.Unit.ENCODER_REV_LIFT, Time.Unit.MINUTE);
        } else {
            return Speed.ZERO;
        }
    }

    public double getCurrent() {
        if(lift != null) {
            return lift.getOutputCurrent();
        }
        return 0;
    }

    public void setPosition(Distance pos) {
        if (lift != null) {
            posSetpoint = pos;
            velSetpoint = Speed.ZERO;

            lift.getPIDController().setReference(pos.get(Distance.Unit.ENCODER_REV_LIFT), ControlType.kPosition, POS_SLOT);
        }

    }

    public void setMotorSpeedRaw(double percent) {
        if (lift != null)
            lift.set(percent);
    }

    public void setMotorSpeedPercent(double percent) {
        if (lift != null)
            setMotorSpeed(MAX_SPEED_LIFT.mul(percent));

    }

    public void setMotorSpeed(Speed speed) {
        if(lift != null) {

            velSetpoint = speed;

            if(EnabledSubsystems.LIFT_DUMB_ENABLED) {
                    lift.set(velSetpoint.div(MAX_SPEED_LIFT));
            } else {
                lift.getPIDController().setReference(velSetpoint.get(Distance.Unit.ENCODER_REV_LIFT, Time.Unit.MINUTE), ControlType.kVelocity, VEL_SLOT);
            }

        }
    }

    public void setVoltageRamp(Time time) {
        lift.setClosedLoopRampRate(time.get(Time.Unit.SECOND));
    }

    private void smartDashboardInit() {
        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putNumber("Lift Pos Setpoint: ", 0);
            SmartDashboard.putNumber("Lift Delta Pos: ", 0);
			SmartDashboard.putNumber("Voltage Ramp Rate Lift Seconds: ",
					VOLTAGE_RAMP_RATE_LIFT.get(Time.Unit.SECOND));
            
            SmartDashboard.putNumber("F Pos Lift: ", F_POS_LIFT);
			SmartDashboard.putNumber("P Pos Lift: ", P_POS_LIFT);
			SmartDashboard.putNumber("I Pos Lift: ", I_POS_LIFT);
            SmartDashboard.putNumber("D Pos Lift: ", D_POS_LIFT);
            SmartDashboard.putNumber("F Vel Lift: ", F_VEL_LIFT);
			SmartDashboard.putNumber("P Vel Lift: ", P_VEL_LIFT);
			SmartDashboard.putNumber("I Vel Lift: ", I_VEL_LIFT);
			SmartDashboard.putNumber("D Vel Lift: ", D_VEL_LIFT);
			
			SmartDashboard.putNumber("Profile Vel Percent Lift: ", profilePercent);
		}
    
    }

    public void smartDashboardInfo() {
        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_BASIC_ENABLED) {
            SmartDashboard.putNumber("Lift Current: ", getCurrent());

            SmartDashboard.putNumberArray("Lift Velocity vs Set Velocity: ", new double[] {getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
        
            SmartDashboard.putNumberArray("Lift Position vs Set Position: ", new double[] {getPosition().get(Distance.Unit.FOOT), posSetpoint.get(Distance.Unit.FOOT)});
        }

        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_DEBUG_ENABLED) {
            setPos = new Distance(SmartDashboard.getNumber("Lift Pos Setpoint: ", 0), Distance.Unit.INCH);
            deltaPos = new Distance(SmartDashboard.getNumber("Lift Delta Pos: ", 0), Distance.Unit.INCH);
			VOLTAGE_RAMP_RATE_LIFT = new Time(SmartDashboard.getNumber("Voltage Ramp Rate Lift Seconds: ",
                    VOLTAGE_RAMP_RATE_LIFT.get(Time.Unit.SECOND)), Time.Unit.SECOND);
            
            F_POS_LIFT = SmartDashboard.getNumber("F Pos Lift: ", F_POS_LIFT);
            lift.getPIDController().setFF(F_POS_LIFT, POS_SLOT);
           
            P_POS_LIFT = SmartDashboard.getNumber("P Pos Lift: ", P_POS_LIFT);
            lift.getPIDController().setP(P_POS_LIFT, POS_SLOT);
           
            I_POS_LIFT = SmartDashboard.getNumber("I Pos Lift: ", I_POS_LIFT);
            lift.getPIDController().setI(I_POS_LIFT, POS_SLOT);
           
            D_POS_LIFT = SmartDashboard.getNumber("D Pos Lift: ", D_POS_LIFT);
            lift.getPIDController().setD(D_POS_LIFT, POS_SLOT);
           
            F_VEL_LIFT = SmartDashboard.getNumber("F Vel Lift: ", F_VEL_LIFT);
            lift.getPIDController().setFF(F_VEL_LIFT, VEL_SLOT);
           
            P_VEL_LIFT = SmartDashboard.getNumber("P Vel Lift: ", P_VEL_LIFT);
            lift.getPIDController().setP(P_VEL_LIFT, VEL_SLOT);
           
            P_VEL_LIFT = SmartDashboard.getNumber("I Vel Lift: ", I_VEL_LIFT);
            lift.getPIDController().setI(I_VEL_LIFT, VEL_SLOT);
           
            P_VEL_LIFT = SmartDashboard.getNumber("D Vel Lift: ", D_VEL_LIFT);
            lift.getPIDController().setD(D_VEL_LIFT, VEL_SLOT);
           
			profilePercent = SmartDashboard.getNumber("Profile Vel Percent Lift: ", profilePercent);

        }

    }

    public void periodic() {

    }

    public void disable() {
        setMotorSpeedRaw(0);
    }
}
