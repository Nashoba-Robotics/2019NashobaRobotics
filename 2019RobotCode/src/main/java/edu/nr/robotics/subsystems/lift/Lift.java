
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
import edu.nr.robotics.subsystems.drive.Drive;
import sun.util.resources.cldr.ext.CurrencyNames_zh_Hant_MO;

public class Lift extends NRSubsystem {

    private static Lift singleton;

    private CANSparkMax liftFront, liftBack;

    public static final double ENC_TICK_PER_INCH_LIFT = 0;

    public static final Speed MAX_SPEED_LIFT_FRONT = Speed.ZERO;
    public static final Speed MAX_SPEED_LIFT_BACK = Speed.ZERO;

    public static final Acceleration MAX_ACCEL_LIFT_FRONT = Acceleration.ZERO;
    public static final Acceleration MAX_ACCEL_LIFT_BACK = Acceleration.ZERO;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT_FRONT = 0;
    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT_BACK = 0;
    
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT_FRONT = 0;
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT_BACK = 0;

    public static final Time VOLTAGE_RAMP_RATE_LIFT = Time.ZERO;

    public static final double P_POS_LIFT_FRONT = 0;
    public static final double I_POS_LIFT_FRONT = 0;
    public static final double D_POS_LIFT_FRONT = 0;

    public static final double P_POS_LIFT_BACK = 0;
    public static final double I_POS_LIFT_BACK = 0;
    public static final double D_POS_LIFT_BACK = 0;

    public static final double P_VEL_LIFT_FRONT = 0;
    public static final double I_VEL_LIFT_FRONT = 0;
    public static final double D_VEL_LIFT_FRONT = 0;

    public static final double P_VEL_LIFT_BACK = 0;
    public static final double I_VEL_LIFT_BACK = 0;
    public static final double D_VEL_LIFT_BACK = 0;

    public static final int PEAK_CURRENT_LIFT = 0;
    public static final int PEAK_CURRENT_DURATION_LIFT = 0;
    public static final int CONTINUOUS_CURRENT_LIMIT_LIFT = 0;

    public static final int VELOCITY_MEASUREMENT_PERIOD_LIFT = 10; // find
    public static final int VELOCITY_MEASUREMENT_WINDOW_LIFT = 32; //find this
    
    public static final int VOLTAGE_COMPENSATION_LEVEL_LIFT = 12;

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
    private Speed frontVelSetpoint = Speed.ZERO;
    private Speed backVelSetpoint = Speed.ZERO;

    private Distance frontPosSetpoint = Distance.ZERO;
    private Distance backPosSetpoint = Distance.ZERO;

    private Lift() {
        if (EnabledSubsystems.LIFT_ENABLED) {
            liftFront = SparkMax.createSpark(RobotMap.LIFT_FRONT, true);
            liftBack = SparkMax.createSpark(RobotMap.LIFT_BACK, true);

            liftFront.getPIDController().setFF(0, VEL_SLOT);
            liftFront.getPIDController().setP(P_VEL_LIFT_FRONT, VEL_SLOT);
            liftFront.getPIDController().setI(I_VEL_LIFT_FRONT, VEL_SLOT);
            liftFront.getPIDController().setD(D_VEL_LIFT_FRONT, VEL_SLOT);

            liftBack.getPIDController().setFF(0, VEL_SLOT);
            liftBack.getPIDController().setP(P_VEL_LIFT_BACK, VEL_SLOT);
            liftBack.getPIDController().setI(I_VEL_LIFT_BACK, VEL_SLOT);
            liftBack.getPIDController().setD(D_VEL_LIFT_BACK, VEL_SLOT);

            liftFront.getPIDController().setFF(0, POS_SLOT);
            liftFront.getPIDController().setP(P_POS_LIFT_FRONT, POS_SLOT);
            liftFront.getPIDController().setI(I_POS_LIFT_FRONT, POS_SLOT);
            liftFront.getPIDController().setD(D_POS_LIFT_FRONT, POS_SLOT);

            liftBack.getPIDController().setFF(0, POS_SLOT);
            liftBack.getPIDController().setP(P_POS_LIFT_BACK, POS_SLOT);
            liftBack.getPIDController().setI(I_POS_LIFT_BACK, POS_SLOT);
            liftBack.getPIDController().setD(D_POS_LIFT_BACK, POS_SLOT);

            liftFront.setIdleMode(IDLE_MODE_LIFT);
            liftBack.setIdleMode(IDLE_MODE_LIFT);

            liftFront.setInverted(false);
            liftBack.setInverted(false);

            liftFront.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT_LIFT);
            liftFront.setSecondaryCurrentLimit(PEAK_CURRENT_LIFT);
            liftBack.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT_LIFT);
            liftBack.setSecondaryCurrentLimit(PEAK_CURRENT_LIFT);

            liftFront.setRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));
            liftBack.setRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));

            smartDashboardInit();

        }

    }

    public static Lift getInstance() {
        if(singleton == null)
            init();
        return singleton;
    }

    public synchronized static void init() {
        if(singleton == null) {
            singleton = new Lift();
            //if(EnabledSubsystems.LIFT_ENABLED)
            //singleton.setJoystickCommand(new LiftJoystickCommand());
        }
    }

    public Distance getFrontPosition() {
        if(liftFront != null){
            return new Distance(liftFront.getEncoder().getPosition(), Distance.Unit.REVOLUTION);
        } else {
            return Distance.ZERO;
        }
    }
}
