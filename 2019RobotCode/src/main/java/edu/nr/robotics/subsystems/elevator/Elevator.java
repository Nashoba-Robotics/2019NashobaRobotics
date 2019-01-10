package edu.nr.robotics.subsystems.elevator;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Time;

public class Elevator extends NRSubsystem implements PIDOutput, PIDSource {

    private static Elevator singleton;

    private TalonSRX elevatorTalon; //follow may be other type of talon
    
    public static final double END_TICK_PER_INCH_CARRIAGE;//find

    public static final Speed MAX_SPEED_ELEVATOR_UP = new Speed();//find
    public static final Speed MAX_SPEED_ELEVATOR_DOWN = new Speed();

    public static final Acceleration MAX_ACCEL_ELEVATOR_UP = new Acceleration();//find
    public static final Acceleration MAX_ACCEL_ELEVATOR_UP = new Acceleration();

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





}