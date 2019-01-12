package edu.nr.robotics.subsystems.elevator;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.lib.motorcontollers.CTRECreator;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.lib.units.Distance;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfiler;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.wpi.first.wpilibj.PIDSourceType;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

public class Elevator extends NRSubsystem implements PIDOutput, PIDSource {

    private static Elevator singleton;

    private TalonSRX elevatorTalon;
    private VictorSPX elevatorTalonFollowOne;
    private VictorSPX elevatorTalonFollowTwo; //follow may be other type of talon
    
    public static final double END_TICK_PER_INCH_CARRIAGE;//find everything

    public static final Speed MAX_SPEED_ELEVATOR_UP = new Speed();//find
    public static final Speed MAX_SPEED_ELEVATOR_DOWN = new Speed();

    public static final Acceleration MAX_ACCEL_ELEVATOR_UP = new Acceleration();//find
    public static final Acceleration MAX_ACCEL_ELEVATOR_DOWN = new Acceleration();

    public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP;

	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN ;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP; //find
    public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN;

    public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_UP ;
    public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN ;
    
    public static Time VOLTAGE_RAMP_RATE_ELEVATOR = Time.ZERO;

    public static double PROFILE_VEL_PERCENT_ELEVATOR ;
    public static final double DROP_PERCENT_ELEVATOR ;
    public static double PROFILE_ACCEL_PERCENT_ELEVATOR ;

    public static double F_POS_ELEVATOR_UP = 0;

    public static double P_POS_ELEVATOR_UP = 0;
    public static double I_POS_ELEVATOR_UP = 0;
    public static double D_POS_ELEVATOR_UP = 0;

    public static double F_POS_ELEVATOR_DOWN = ((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN * MAX_SPEED_ELEVATOR_DOWN.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
    + MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN) * 1023.0)
    / MAX_SPEED_ELEVATOR_DOWN.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
    Time.Unit.HUNDRED_MILLISECOND);

    public static double P_POS_ELEVATOR_DOWN = 0; // Find elevator MagicMotion PID values
	public static double I_POS_ELEVATOR_DOWN = 0;
	public static double D_POS_ELEVATOR_DOWN = 0;

    public static double P_VEL_ELEVATOR_UP = 0;
	public static double I_VEL_ELEVATOR_UP = 0;
	public static double D_VEL_ELEVATOR_UP = 0;

	public static double P_VEL_ELEVATOR_DOWN = 0; //  Find elevator velocity PID values for down
	public static double I_VEL_ELEVATOR_DOWN = 0;
	public static double D_VEL_ELEVATOR_DOWN = 0;


    public static final Distance PROFILE_END_POS_THRESHOLD_ELEVATOR = new Distance(3, Distance.Unit.INCH);
    public static final Speed PROFILE_STOP_SPEED_THRESHOLD = new Speed(0.1, Distance.Unit.INCH, Time.Unit.SECOND);
    
    
    public static final double RAMPED_PROFILE_TIME_MULT_ELEVATOR = 1000;

    public static final int PEAK_CURRENT_ELEVATOR = 80;
    public static final int PEAK_CURRENT_DURATION_ELEVATOR = 1000;
    public static final int CONTINUOUS_CURRENT_LIMIT_ELEVATOR = 40;

    public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_ELEVATOR = VelocityMeasPeriod.Period_10Ms;
    public static final double VELOCITY_MEASUREMENT_WINDOW_ELEVATOR = 32;
    
    public static final double VOLTAGE_COMPENSATION_LEVEL_ELEVATOR = 12;

    public static final NeutralMode NEUTRAL_MODE_ELEVATOR = NeutralMode.Brake;

    public static final int PID_TYPE = 0;

    public static final int DEFAULT_TIMEOUT = 0;

    public static final int VEL_UP_SLOT = 0;
    public static final int MOTION_MAGIC_UP_SLOT = 1;//figure out for real
    public static final int VEL_DOWN_SLOT = 2;
    public static final int MOTION_MAGIC_DOWN_SLOT = 3;

    public static final double kV_UP = 1 / MAX_SPEED_ELEVATOR_UP.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
    public static double kA_DOWN = 0;
    public static double kP_DOWN = 0;
    public static double kD_DOWN = 0;

    public static final Distance TOP_HEIGHT_ELEVATOR = new Distance();//find these
    public static final Distance HATCH_PICKUP_GROUND_ELEVATOR = new Distance();
    public static final Distance HATCH_PLACE_LOW_ELEVATOR = new Distance();
    public static final Distance HATCH_PLACE_MIDDLE_ELEVATOR = new Distance();
    public static final Distance HATCH_PLACE_TOp_ELEVATOR = new Distance();//find these
    public static final Distance CARGO_PLACE_LOW_ELEVATOR = new Distance();
    public static final Distance CARGO_PLACE_MIDDLE_ELEVATOR = new Distance();
    public static final Distance CARGO_PLACE_TOP_ELEVATOR = new Distance();

    private Speed velSetpoint = Speed.ZERO;
    private Distance posSetpoint = Distance.ZERO;

    public static Distance profilePos = Distance.ZERO;

    public PIDSourceType type = PIDSourceType.kRate;

    public OneDimensionalMotionProfilerBasic basicProfiler;

    private Elevator() {
        if(EnabledSubsystems.ELEVATOR_ENABLED) {
            elevatorTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_TALON);
            elevatorTalonFollowOne = CTRECreator.createFollowerVictor(RobotMap.ELEVATOR_FOLLOW_ONE, elevatorTalon);
            elevatorTalonFollowTwo = CTRECreator.createFollowerVictor(RobotMap.ELEVATOR_FOLLOW_TWO, elevatorTalon);

        }
    }




}