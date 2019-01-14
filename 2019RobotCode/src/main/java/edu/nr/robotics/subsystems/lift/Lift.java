
package edu.nr.robotics.subsystems.lift;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;

public class Lift extends NRSubsystem {

    public static final double ENC_TICK_PER_INCH_LIFT = 0;

    public static final Speed MAX_SPEED_LIFT = Speed.ZERO;

    public static final Acceleration MAX_ACCEL_LIFT = Acceleration.ZERO;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT = 0;
    
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT = 0;

    public static final double VOLTAGE_RAMP_RATE_LIFT = 0;

    public static final double P_POS_LIFT = 0;
    public static final double I_POS_LIFT = 0;
    public static final double D_POS_LIFT = 0;

    public static final double P_Vel_LIFT = 0;
    public static final double I_Vel_LIFT = 0;
    public static final double D_Vel_LIFT = 0;

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

    public static final Distance TOP_POSITION = Distance.ZERO;
    public static final Distance LEVEL1_POS = Distance.ZERO;
    public static final Distance LEVEL2_POS = Distance.ZERO;
    public static final Distance BOTTOM_HEIGHT = Distance.ZERO;

    //tracking drive motor setpoints
    private Speed frontVelSetpoint = Speed.ZERO;
    private Speed backVelSetpoint = Speed.ZERO;

    private Distance frontPosSetpoint = Distance.ZERO;
    private Distance backPosSetpoint = Distance.ZERO;

}
