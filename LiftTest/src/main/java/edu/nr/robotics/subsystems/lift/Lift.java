
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

    private CANSparkMax /*liftFront,*/ liftBack;

    public static final double INCH_PER_REVOLUTION_LIFT = 1;

    public static final Speed MAX_SPEED_LIFT = new Speed(8.52, Distance.Unit.FOOT, Time.Unit.SECOND);

    public static final Acceleration MAX_ACCEL_LIFT_FRONT = Acceleration.ZERO;
    public static final Acceleration MAX_ACCEL_LIFT_BACK = Acceleration.ZERO;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT_FRONT = 0.00332;
    public static final double MIN_MOVE_VOLTAGE_PERCENT_LIFT_BACK = 0.00175;
    
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT_FRONT = 0.117;
    public static final double VOLTAGE_VELOCITY_SLOPE_LIFT_BACK = 0.115;

    public static Time VOLTAGE_RAMP_RATE_LIFT = Time.ZERO;

    public static double F_POS_LIFT_FRONT = 0; //Leave F at 0
    public static double P_POS_LIFT_FRONT = 0.1;
    public static double I_POS_LIFT_FRONT = 0;
    public static double D_POS_LIFT_FRONT = 0;

    public static double F_POS_LIFT_BACK = 0; //Leave F at 0
    public static double P_POS_LIFT_BACK = 0.1;
    public static double I_POS_LIFT_BACK = 0;
    public static double D_POS_LIFT_BACK = 0;

    public static double F_VEL_LIFT_FRONT = 0.000175;
    public static double P_VEL_LIFT_FRONT = 0.0005;
    public static double I_VEL_LIFT_FRONT = 0;
    public static double D_VEL_LIFT_FRONT = 0;

    public static double F_VEL_LIFT_BACK = 0.000165;
    public static double P_VEL_LIFT_BACK = 0.0004;
    public static double I_VEL_LIFT_BACK = 0;
    public static double D_VEL_LIFT_BACK = 0;

    public static final int PEAK_CURRENT_LIFT = 60;
    //public static final int PEAK_CURRENT_DURATION_LIFT = 250;
    public static final int CONTINUOUS_CURRENT_LIMIT_LIFT = 40;

    public static double PROFILE_VEL_PERCENT_LIFT = 0.2;

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

    public static Distance frontSetPos = Distance.ZERO;
    public static Distance backSetPos = Distance.ZERO;
    public static Speed frontSetVel = Speed.ZERO;
    public static Speed backSetVel = Speed.ZERO;
    public static double frontPercent = 0.1;
    public static double backPercent = 0.1;

    private Lift() {
        if (EnabledSubsystems.LIFT_ENABLED) {
            //liftFront = SparkMax.createSpark(RobotMap.LIFT_FRONT, true);
            liftBack = SparkMax.createSpark(RobotMap.LIFT_BACK, true);

            /*liftFront.getPIDController().setFF(F_VEL_LIFT_FRONT, VEL_SLOT);
            liftFront.getPIDController().setP(P_VEL_LIFT_FRONT, VEL_SLOT);
            liftFront.getPIDController().setI(I_VEL_LIFT_FRONT, VEL_SLOT);
            liftFront.getPIDController().setD(D_VEL_LIFT_FRONT, VEL_SLOT);*/

            liftBack.getPIDController().setFF(F_VEL_LIFT_BACK, VEL_SLOT);
            liftBack.getPIDController().setP(P_VEL_LIFT_BACK, VEL_SLOT);
            liftBack.getPIDController().setI(I_VEL_LIFT_BACK, VEL_SLOT);
            liftBack.getPIDController().setD(D_VEL_LIFT_BACK, VEL_SLOT);

            /*liftFront.getPIDController().setFF(F_POS_LIFT_FRONT, POS_SLOT);
            liftFront.getPIDController().setP(P_POS_LIFT_FRONT, POS_SLOT);
            liftFront.getPIDController().setI(I_POS_LIFT_FRONT, POS_SLOT);
            liftFront.getPIDController().setD(D_POS_LIFT_FRONT, POS_SLOT);*/

            liftBack.getPIDController().setFF(F_POS_LIFT_BACK, POS_SLOT);
            liftBack.getPIDController().setP(P_POS_LIFT_BACK, POS_SLOT);
            liftBack.getPIDController().setI(I_POS_LIFT_BACK, POS_SLOT);
            liftBack.getPIDController().setD(D_POS_LIFT_BACK, POS_SLOT);

            //liftFront.setIdleMode(IDLE_MODE_LIFT);
            liftBack.setIdleMode(IDLE_MODE_LIFT);

            //liftFront.setInverted(false);
            liftBack.setInverted(false);

            //liftFront.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT_LIFT);
            //liftFront.setSecondaryCurrentLimit(PEAK_CURRENT_LIFT);
            liftBack.setSmartCurrentLimit(CONTINUOUS_CURRENT_LIMIT_LIFT);
            liftBack.setSecondaryCurrentLimit(PEAK_CURRENT_LIFT);

            //liftFront.setRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));
            liftBack.setRampRate(VOLTAGE_RAMP_RATE_LIFT.get(Unit.SECOND));

            //liftFront.getPIDController().setOutputRange(-1, 1, VEL_SLOT);
            liftBack.getPIDController().setOutputRange(-1, 1, VEL_SLOT);
            //liftFront.getPIDController().setOutputRange(-1, 1, POS_SLOT);
            liftBack.getPIDController().setOutputRange(-1, 1, POS_SLOT);

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

    /*public Distance getFrontPosition() {
        if(liftFront != null) {
            return new Distance(liftFront.getEncoder().getPosition(), Distance.Unit.REVOLUTION_LIFT);
        } else {
            return Distance.ZERO;
        }
    }*/

    public Distance getBackPosition() {
        if(liftBack != null){
            return new Distance(liftBack.getEncoder().getPosition(), Distance.Unit.REVOLUTION_LIFT);
        } else {
            return Distance.ZERO;
        }
    }

    /*public Speed getFrontVelocity() {
        if(liftFront != null){
            return new Speed(liftFront.getEncoder().getVelocity(), Distance.Unit.REVOLUTION_LIFT, Time.Unit.MINUTE);
        } else {
            return Speed.ZERO;
        }
    }*/

    public Speed getBackVelocity() {
        if(liftBack != null){
            return new Speed(liftBack.getEncoder().getVelocity(), Distance.Unit.REVOLUTION_LIFT, Time.Unit.MINUTE);
        } else {
            return Speed.ZERO;
        }
    }

    /*public double getFrontCurrent() {
        if(liftFront != null) {
            return liftFront.getOutputCurrent();
        }
        return 0;
    }*/

    public double getBackCurrent() {
        if(liftBack != null) {
            return liftBack.getOutputCurrent();
        }
        return 0;
    }

    public void setPosition(Distance frontPos, Distance backPos) {
        if (/*liftFront != null &&*/ liftBack != null) {
            frontPosSetpoint = frontPos;
            backPosSetpoint = backPos;
            frontVelSetpoint = Speed.ZERO;
            backVelSetpoint = Speed.ZERO;

            //System.out.println("FF: " + liftFront.getPIDController().getFF(POS_SLOT));
            //liftFront.getPIDController().setReference(frontPos.get(Distance.Unit.REVOLUTION_LIFT), ControlType.kPosition, POS_SLOT);//, liftFront.getPIDController().getFF(POS_SLOT));
            liftBack.getPIDController().setReference(backPos.get(Distance.Unit.REVOLUTION_LIFT), ControlType.kPosition, POS_SLOT);//, liftBack.getPIDController().getFF(POS_SLOT));
        }

    }

    public void setMotorSpeedRaw(double front, double back) {
        //liftFront.set(front);
        liftBack.set(back);
    }

    public void setMotorSpeedPercent(double front, double back) {
        setMotorSpeed(MAX_SPEED_LIFT.mul(front), MAX_SPEED_LIFT.mul(back));

    }

    public void setMotorSpeed(Speed front, Speed back) {
        if(/*liftFront != null &&*/ liftBack != null) {

            frontVelSetpoint = front;
            backVelSetpoint = back;
            //liftFront.getPIDController().setFF(((VOLTAGE_VELOCITY_SLOPE_LIFT_FRONT * frontVelSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LIFT_FRONT) * 1/335), VEL_SLOT); //THESE WILL NOT WORK AT ALL. BUT IF THEY DO, NATHANIEL IS RESPONSIBLE. IF THEY DON'T, ETHAN IS RESPONSIBLE.
            //liftBack.getPIDController().setFF(((VOLTAGE_VELOCITY_SLOPE_LIFT_BACK * backVelSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LIFT_BACK) * 1/335), VEL_SLOT);

            if(EnabledSubsystems.LIFT_DUMB_ENABLED) {
                    //liftFront.set(front.div(MAX_SPEED_LIFT));
                    liftBack.set(back.div(MAX_SPEED_LIFT));
            } else {              
                    //liftFront.getPIDController().setReference(front.get(Distance.Unit.REVOLUTION_LIFT, Time.Unit.MINUTE), ControlType.kVelocity, VEL_SLOT);//, liftFront.getPIDController().getFF(VEL_SLOT));
                    liftBack.getPIDController().setReference(back.get(Distance.Unit.REVOLUTION_LIFT, Time.Unit.MINUTE), ControlType.kVelocity, VEL_SLOT);//, liftBack.getPIDController().getFF(VEL_SLOT));
            }

        }
    }

    public void setVoltageRamp(Time time) {
        //liftFront.setRampRate(time.get(Time.Unit.SECOND));
        liftBack.setRampRate(time.get(Time.Unit.SECOND));
    }

    private void smartDashboardInit() {
        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putNumber("Lift Front Pos Setpoint: ", 0);
            SmartDashboard.putNumber("Lift Back Pos Setpoint: ", 0);
            SmartDashboard.putNumber("Lift Front Vel Setpoint: ", 0);
            SmartDashboard.putNumber("Lift Back Vel Setpoint: ", 0);
			SmartDashboard.putNumber("Voltage Ramp Rate Lift Seconds: ",
					VOLTAGE_RAMP_RATE_LIFT.get(Time.Unit.SECOND));
            
            SmartDashboard.putNumber("F Pos Lift Back: ", F_POS_LIFT_BACK);
			SmartDashboard.putNumber("P Pos Lift Back: ", P_POS_LIFT_BACK);
			SmartDashboard.putNumber("I Pos Lift Back: ", I_POS_LIFT_BACK);
            SmartDashboard.putNumber("D Pos Lift Back: ", D_POS_LIFT_BACK);
            SmartDashboard.putNumber("F Vel Lift Back: ", F_POS_LIFT_BACK);
			SmartDashboard.putNumber("P Vel Lift Back: ", P_VEL_LIFT_BACK);
			SmartDashboard.putNumber("I Vel Lift Back: ", I_VEL_LIFT_BACK);
			SmartDashboard.putNumber("D Vel Lift Back: ", D_VEL_LIFT_BACK);
            
            SmartDashboard.putNumber("F Pos Lift Front: ", F_POS_LIFT_FRONT);
			SmartDashboard.putNumber("P Pos Lift Front: ", P_POS_LIFT_FRONT);
			SmartDashboard.putNumber("I Pos Lift Front: ", I_POS_LIFT_FRONT);
            SmartDashboard.putNumber("D Pos Lift Front: ", D_POS_LIFT_FRONT);
            SmartDashboard.putNumber("F Vel Lift Front: ", F_VEL_LIFT_FRONT);
			SmartDashboard.putNumber("P Vel Lift Front: ", P_VEL_LIFT_FRONT);
			SmartDashboard.putNumber("I Vel Lift Front: ", I_VEL_LIFT_FRONT);
			SmartDashboard.putNumber("D Vel Lift Front: ", D_VEL_LIFT_FRONT);
			
            SmartDashboard.putNumber("Profile Vel Percent Lift: ", PROFILE_VEL_PERCENT_LIFT);
            
            SmartDashboard.putNumber("Lift Front Percent: ", frontPercent);
            SmartDashboard.putNumber("Lift Back Percent: ", backPercent);
		}
    
    }

    public void smartDashboardInfo() {
        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_BASIC_ENABLED) {
            //SmartDashboard.putNumber("Lift Front Current: ", getFrontCurrent());
            SmartDashboard.putNumber("Lift Back Current: ", getBackCurrent());

            //SmartDashboard.putNumberArray("Lift Front Velocity vs Set Velocity: ", new double[] {getFrontVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), frontVelSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
            SmartDashboard.putNumberArray("Lift Back Velocity vs Set Velocity: ", new double[] {getBackVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), backVelSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
        
            //SmartDashboard.putNumberArray("Lift Front Position vs Set Position: ", new double[] {getFrontPosition().get(Distance.Unit.FOOT), frontPosSetpoint.get(Distance.Unit.FOOT)});
            SmartDashboard.putNumberArray("Lift Back Position vs Set Position: ", new double[] {getBackPosition().get(Distance.Unit.FOOT), backPosSetpoint.get(Distance.Unit.FOOT)});
        
        }

        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_DEBUG_ENABLED) {
            frontSetPos = new Distance(SmartDashboard.getNumber("Lift Front Pos Setpoint: ", 0), Distance.Unit.FOOT);
            backSetPos = new Distance(SmartDashboard.getNumber("Lift Back Pos Setpoint: ", 0), Distance.Unit.FOOT);
            frontSetVel = new Speed(SmartDashboard.getNumber("Lift Front Vel Setpoint: ", 0), Distance.Unit.FOOT, Time.Unit.SECOND);
            backSetVel = new Speed(SmartDashboard.getNumber("Lift Back Vel Setpoint: ", 0), Distance.Unit.FOOT, Time.Unit.SECOND);
			VOLTAGE_RAMP_RATE_LIFT = new Time(SmartDashboard.getNumber("Voltage Ramp Rate Lift Seconds: ",
					VOLTAGE_RAMP_RATE_LIFT.get(Time.Unit.SECOND)), Time.Unit.SECOND);
            
            F_POS_LIFT_BACK = SmartDashboard.getNumber("F Pos Lift Back: ", F_POS_LIFT_BACK);
            liftBack.getPIDController().setFF(F_POS_LIFT_BACK, POS_SLOT);
            P_POS_LIFT_BACK = SmartDashboard.getNumber("P Pos Lift Back: ", P_POS_LIFT_BACK);
            liftBack.getPIDController().setP(P_POS_LIFT_BACK, POS_SLOT);
            I_POS_LIFT_BACK = SmartDashboard.getNumber("I Pos Lift Back: ", I_POS_LIFT_BACK);
            liftBack.getPIDController().setI(I_POS_LIFT_BACK, POS_SLOT);
            D_POS_LIFT_BACK = SmartDashboard.getNumber("D Pos Lift Back: ", D_POS_LIFT_BACK);
            liftBack.getPIDController().setD(D_POS_LIFT_BACK, POS_SLOT);
            F_VEL_LIFT_BACK = SmartDashboard.getNumber("F Vel Lift Back: ", F_VEL_LIFT_BACK);
            liftBack.getPIDController().setFF(F_VEL_LIFT_BACK, VEL_SLOT);
            P_VEL_LIFT_BACK = SmartDashboard.getNumber("P Vel Lift Back: ", P_VEL_LIFT_BACK);
            liftBack.getPIDController().setP(P_VEL_LIFT_BACK, VEL_SLOT);
            P_VEL_LIFT_BACK = SmartDashboard.getNumber("I Vel Lift Back: ", I_VEL_LIFT_BACK);
            liftBack.getPIDController().setI(I_VEL_LIFT_BACK, VEL_SLOT);
            P_VEL_LIFT_BACK = SmartDashboard.getNumber("D Vel Lift Back: ", D_VEL_LIFT_BACK);
            liftBack.getPIDController().setD(D_VEL_LIFT_BACK, VEL_SLOT);
            
            /*F_POS_LIFT_FRONT = SmartDashboard.getNumber("F Pos Lift Front: ", F_POS_LIFT_FRONT);
            liftFront.getPIDController().setFF(F_POS_LIFT_FRONT, POS_SLOT);
            P_POS_LIFT_FRONT = SmartDashboard.getNumber("P Pos Lift Front: ", P_POS_LIFT_FRONT);
            liftFront.getPIDController().setP(P_POS_LIFT_FRONT, POS_SLOT);
            I_POS_LIFT_FRONT = SmartDashboard.getNumber("I Pos Lift Front: ", I_POS_LIFT_FRONT);
            liftFront.getPIDController().setI(I_POS_LIFT_FRONT, POS_SLOT);
            D_POS_LIFT_FRONT = SmartDashboard.getNumber("D Pos Lift Front: ", D_POS_LIFT_FRONT);
            liftFront.getPIDController().setD(D_POS_LIFT_FRONT, POS_SLOT);
            F_VEL_LIFT_FRONT = SmartDashboard.getNumber("F Vel Lift Front: ", F_VEL_LIFT_FRONT);
            liftFront.getPIDController().setFF(F_VEL_LIFT_FRONT, VEL_SLOT);
            P_VEL_LIFT_FRONT = SmartDashboard.getNumber("P Vel Lift Front: ", P_VEL_LIFT_FRONT);
            liftFront.getPIDController().setP(P_VEL_LIFT_FRONT, VEL_SLOT);
            I_VEL_LIFT_FRONT = SmartDashboard.getNumber("I Vel Lift Front: ", I_VEL_LIFT_FRONT);
            liftFront.getPIDController().setI(I_VEL_LIFT_FRONT, VEL_SLOT);
            D_VEL_LIFT_FRONT = SmartDashboard.getNumber("D Vel Lift Front: ", D_VEL_LIFT_FRONT);
            liftFront.getPIDController().setD(D_VEL_LIFT_FRONT, VEL_SLOT);*/
			
            PROFILE_VEL_PERCENT_LIFT = SmartDashboard.getNumber("Profile Vel Percent Lift: ", PROFILE_VEL_PERCENT_LIFT);
            
            frontPercent = SmartDashboard.getNumber("Lift Front Percent: ", frontPercent);
            backPercent = SmartDashboard.getNumber("Lift Back Percent: ", backPercent);

        }

    }

    public void periodic() {

    }

    public void disable() {
        setMotorSpeedRaw(0, 0);
    }
}
