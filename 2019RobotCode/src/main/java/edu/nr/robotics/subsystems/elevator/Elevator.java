package edu.nr.robotics.subsystems.elevator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.nr.lib.units.Distance;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfiler;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectoryRamped;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.motorcontrollers.CTRECreator;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.nr.robotics.OI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

public class Elevator extends NRSubsystem implements PIDOutput, PIDSource {

    private static Elevator singleton;

    private TalonSRX elevatorTalon;
    private VictorSPX elevatorVictorFollowOne;
    private VictorSPX elevatorVictorFollowTwo; //follow may be other type of talon
    private PowerDistributionPanel pdp;
    
    private DoubleSolenoid gearShifter;

    public static final double ENC_TICK_PER_INCH_CARRIAGE = 0; //find everything, these are 2018 numbers for testing

    public static final Speed MAX_SPEED_ELEVATOR_UP = Speed.ZERO;//find
    public static final Speed MAX_SPEED_ELEVATOR_DOWN = Speed.ZERO;

    public static final Acceleration MAX_ACCEL_ELEVATOR_UP = Acceleration.ZERO;//find
    public static final Acceleration MAX_ACCEL_ELEVATOR_DOWN = Acceleration.ZERO;

    public static final Speed MAX_CLIMB_SPEED_UP = Speed.ZERO;
    public static final Speed MAX_CLIMB_SPEED_DOWN = Speed.ZERO;

    public static final Acceleration MAX_CLIMB_ACCEL_UP = Acceleration.ZERO;
    public static final Acceleration MAX_CLIMB_ACCEL_DOWN = Acceleration.ZERO;

    public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP = 0.;

	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN = 0.;

    public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP = 0.; //find
    public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN = 0;
    public static final double MIN_MOVE_VOLTAGE_PERCENT_CLIMB_UP = 0;

    public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_UP = 0;
    public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN = 0;
    public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_CLIMB_UP = 0;
    
    public static Time VOLTAGE_RAMP_RATE_ELEVATOR = Time.ZERO;

    public static double PROFILE_VEL_PERCENT_ELEVATOR = 0.9;
    public static final double DROP_PERCENT_ELEVATOR = -0.4;
    public static double PROFILE_ACCEL_PERCENT_ELEVATOR = 0.9;

    public static double F_POS_ELEVATOR_UP = 0.;

    public static double P_POS_ELEVATOR_UP = 0.0;
    public static double I_POS_ELEVATOR_UP = 0;
    public static double D_POS_ELEVATOR_UP = 0.0;

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
    
    public static double F_POS_CLIMB_UP = 0;
    public static double P_POS_CLIMB_UP = 0;
	public static double I_POS_CLIMB_UP = 0;
	public static double D_POS_CLIMB_UP = 0;


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

    public static final int VEL_ELEV_UP_SLOT = 0;
    public static final int MOTION_MAGIC_ELEV_UP_SLOT = 1;//figure out for real
    public static final int MOTION_MAGIC_CLIMB_UP_SLOT = 2;
    public static final int VEL_CLIMB_UP_SLOT = 3;

    public static final double kV_UP = 1 / MAX_SPEED_ELEVATOR_UP.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
    public static double kA_UP = 0;
	public static double kP_UP = 0;
	public static double kD_UP = 0;

	public static final double kV_DOWN = 1 / MAX_SPEED_ELEVATOR_DOWN.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
    public static double kA_DOWN = 0;
    public static double kP_DOWN = 0;
    public static double kD_DOWN = 0;

    public static final Distance TOP_HEIGHT_ELEVATOR = Distance.ZERO;//find these
    public static final Distance HATCH_PICKUP_GROUND_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance HATCH_PLACE_LOW_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance HATCH_PLACE_MIDDLE_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance HATCH_PLACE_TOP_HEIGHT_ELEVATOR = Distance.ZERO;//find these
    public static final Distance CARGO_PLACE_LOW_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance CARGO_PLACE_MIDDLE_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance CARGO_PLACE_TOP_HEIGHT_ELEVATOR = Distance.ZERO;
    public static final Distance CARGO_PICKUP_HEIGHT_ELEVATOR = Distance.ZERO;

    private Speed velSetpoint = Speed.ZERO;
    private Distance posSetpoint = Distance.ZERO;

    public static Distance profilePos = Distance.ZERO;

    public PIDSourceType type = PIDSourceType.kRate;

    public OneDimensionalMotionProfilerBasic basicProfiler;

    public static enum Gear {
		elevator, climb;

		// TODO: Drive: Find which gear directions are forward/reverse
		private static Value ELEVATOR_VALUE = Value.kForward;
		private static Value CLIMB_VALUE = Value.kReverse;

		private static int ELEVATOR_PROFILE = 0;
		private static int CLIMB_PROFILE = 1;
    }

    private int getCurrentGearProfile() {
		if (getCurrentGear() == Gear.elevator) {
			return Gear.ELEVATOR_PROFILE;
		} else {
			return Gear.CLIMB_PROFILE;
		}
	}

	public Speed currentMaxSpeed() {
		if (getCurrentGear() == Gear.elevator) {
			return MAX_SPEED_ELEVATOR_UP;
		} else {
			return MAX_CLIMB_SPEED_UP;
		}
	}
    
    private Elevator() {
        if(EnabledSubsystems.ELEVATOR_ENABLED) {
            elevatorTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_TALON);
            elevatorVictorFollowOne = CTRECreator.createFollowerVictor(RobotMap.ELEVATOR_FOLLOW_ONE, elevatorTalon);
            elevatorVictorFollowTwo = CTRECreator.createFollowerVictor(RobotMap.ELEVATOR_FOLLOW_TWO, elevatorTalon);
            pdp = new PowerDistributionPanel();

            gearShifter = new DoubleSolenoid(RobotMap.ELEVATOR_GEAR_SWITCHER_PCM_PORT,
            RobotMap.ELEVATOR_GEAR_SWITCHER_FORWARD_CHANNEL, RobotMap.ELEVATOR_GEAR_SWITCHER_REVERSE_CHANNEL);
            
            elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
        
            elevatorTalon.config_kF(VEL_ELEV_UP_SLOT, 0, DEFAULT_TIMEOUT);
            elevatorTalon.config_kP(VEL_ELEV_UP_SLOT, P_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kI(VEL_ELEV_UP_SLOT, I_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kD(VEL_ELEV_UP_SLOT, D_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kF(MOTION_MAGIC_ELEV_UP_SLOT, F_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kP(MOTION_MAGIC_ELEV_UP_SLOT, P_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kI(MOTION_MAGIC_ELEV_UP_SLOT, I_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kD(MOTION_MAGIC_ELEV_UP_SLOT, D_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);

            elevatorTalon.config_kF(MOTION_MAGIC_CLIMB_UP_SLOT, F_POS_CLIMB_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kP(MOTION_MAGIC_CLIMB_UP_SLOT, P_POS_CLIMB_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kI(MOTION_MAGIC_CLIMB_UP_SLOT, I_POS_CLIMB_UP, DEFAULT_TIMEOUT);
            elevatorTalon.config_kD(MOTION_MAGIC_CLIMB_UP_SLOT, D_POS_CLIMB_UP, DEFAULT_TIMEOUT);

            elevatorTalon.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
            elevatorVictorFollowOne.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
            elevatorVictorFollowTwo.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
            
            elevatorTalon.setInverted(false);
            elevatorTalon.setSensorPhase(true);
            elevatorVictorFollowOne.setInverted(false);
            elevatorVictorFollowTwo.setInverted(false);

            elevatorTalon.enableVoltageCompensation(true);
            elevatorTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_ELEVATOR, DEFAULT_TIMEOUT);
            
            elevatorTalon.enableCurrentLimit(true);
            elevatorTalon.configPeakCurrentLimit(PEAK_CURRENT_ELEVATOR, DEFAULT_TIMEOUT);
            elevatorTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_ELEVATOR, DEFAULT_TIMEOUT);
			//elevTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_ELEVATOR, DEFAULT_TIMEOUT);

            elevatorTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
            elevatorTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);

            elevatorTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
                Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
                DEFAULT_TIMEOUT);

            elevatorTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
				Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
                DEFAULT_TIMEOUT);
                


            elevatorTalon.getSensorCollection().setQuadraturePosition(0, DEFAULT_TIMEOUT);

            if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
                elevatorTalon.set(ControlMode.PercentOutput, 0);
            } else {
                elevatorTalon.set(ControlMode.Velocity, 0);
            }

        }

        smartDashboardInit();

    }

    	/**
	 * Sets the current talon profile
	 * 
	 * @param profile
	 */
	private void setProfile(int profileSlot) {
		if (elevatorTalon != null)
			elevatorTalon.selectProfileSlot(profileSlot, DEFAULT_TIMEOUT);
	}

    public void switchToElevGear() {
		setProfile(Gear.ELEVATOR_PROFILE);
		if (gearShifter != null) {
			gearShifter.set(Gear.ELEVATOR_VALUE);
		}
	}

	public void switchToClimbGear() {
		setProfile(Gear.CLIMB_PROFILE);
		if (gearShifter != null) {
			gearShifter.set(Gear.ELEVATOR_VALUE);
		}
	}

	public Gear getCurrentGear() {
		if (gearShifter != null) {
			if (gearShifter.get() == Gear.ELEVATOR_VALUE) {
				return Gear.elevator;
			} else {
				return Gear.climb;
			}
		} else {
			return Gear.elevator;
		}
	}

	public void switchGear() {
		if (getCurrentGear() == Gear.climb) {
			switchToElevGear();
		} else {
			switchToClimbGear();
		}
	}

    public static Elevator getInstance() {
        if(singleton == null)
            init();
            return singleton;


    }

    public synchronized static void init() {
        if (singleton == null) {
            singleton = new Elevator();
            singleton.setJoystickCommand(new ElevatorJoystickCommand());

        }
    }

    public Distance getPosition() {
        if (elevatorTalon != null)
            return new Distance(elevatorTalon.getSelectedSensorPosition(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV);
        return Distance.ZERO;    
    }

    public Speed getVelocity() {
        if (elevatorTalon != null)
        return new Speed(elevatorTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
            Time.Unit.HUNDRED_MILLISECOND);
        return Speed.ZERO;
    }

    public double getMasterCurrent() {
        if (elevatorTalon != null)
            return elevatorTalon.getOutputCurrent();
        return 0;
    }

    public double getFollowOneCurrent() {
        if (elevatorVictorFollowOne != null)
            return pdp.getCurrent(RobotMap.ELEVATOR_FOLLOW_ONE_CURRENT);
        return 0;
    }

    public double getFollowTwoCurrent() {
        if (elevatorVictorFollowTwo != null)
            return pdp.getCurrent(RobotMap.ELEVATOR_FOLLOW_TWO_CURRENT);
        return 0;
    }

    public void setPosition(Distance position) {
            posSetpoint = position;
			velSetpoint = Speed.ZERO;
            
            if (getCurrentGear() == Gear.elevator) {     
                elevatorTalon.selectProfileSlot(MOTION_MAGIC_ELEV_UP_SLOT, DEFAULT_TIMEOUT);
                
                elevatorTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
                        Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
                                DEFAULT_TIMEOUT);
                elevatorTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
                        Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
                        DEFAULT_TIMEOUT);
                
                elevatorTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV));
                
            }
            else if (getCurrentGear() == Gear.climb) { 
                elevatorTalon.selectProfileSlot(MOTION_MAGIC_CLIMB_UP_SLOT, DEFAULT_TIMEOUT);
                
                elevatorTalon.configMotionCruiseVelocity((int) MAX_CLIMB_SPEED_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
                        Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
                                DEFAULT_TIMEOUT);
                elevatorTalon.configMotionAcceleration((int) MAX_CLIMB_ACCEL_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
                        Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
                        DEFAULT_TIMEOUT);
                
                elevatorTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV));
                    
            }
        }
    


    public void setMotorPercentRaw(double percent) {
        if (elevatorTalon != null) {
            elevatorTalon.set(ControlMode.PercentOutput, percent);

        }

    }

    public void setMotorSpeedPercent(double percent) {
        if (elevatorTalon != null) {
            //elevTalon.set(ControlMode.PercentOutput, percent);
            if(getCurrentGear() == Gear.elevator)
                setMotorSpeed(MAX_SPEED_ELEVATOR_UP.mul(percent));
            else
                setMotorSpeed(MAX_CLIMB_SPEED_UP.mul(percent));
        }
    }

    public void setMotorSpeed(Speed speed) {

        if(elevatorTalon != null) {

            velSetpoint = speed;
            posSetpoint = Distance.ZERO;

            if (getCurrentGear() == Gear.climb) {
                elevatorTalon.selectProfileSlot(VEL_CLIMB_UP_SLOT, DEFAULT_TIMEOUT);

                elevatorTalon.config_kF(VEL_CLIMB_UP_SLOT,
                    ((VOLTAGE_PERCENT_VELOCITY_SLOPE_CLIMB_UP * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
                            + MIN_MOVE_VOLTAGE_PERCENT_CLIMB_UP) * 1023.0)
                            / velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
                                    Time.Unit.HUNDRED_MILLISECOND),
                    DEFAULT_TIMEOUT);

                elevatorTalon.selectProfileSlot(VEL_ELEV_UP_SLOT, DEFAULT_TIMEOUT);
            } else if (getCurrentGear() == Gear.elevator) {
                elevatorTalon.config_kF(VEL_ELEV_UP_SLOT,
                    ((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_UP * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
                            + MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP) * 1023.0)
                            / velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
                                    Time.Unit.HUNDRED_MILLISECOND),
                    DEFAULT_TIMEOUT);
            }

            if(EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
                elevatorTalon.set(ControlMode.PercentOutput, velSetpoint.div(MAX_SPEED_ELEVATOR_UP));
            } else {
                elevatorTalon.set(ControlMode.Velocity,
                    velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND));
            }
        }
    }

    public void setVoltageRamp(Time time) {
        if (elevatorTalon != null) {
            elevatorTalon.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
            elevatorTalon.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);


        }
    }
    public void enableMotionProfiler(Distance dist,double maxVelPercent, double maxAccelPercent) {
        Distance tempDist = dist;//.mul(1.227).add(new Distance(-4.533, Distance.Unit.INCH));

        if (dist.greaterThan(Distance.ZERO)) {

			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_UP, kA_UP, kP_UP, kD_UP);
			basicProfiler.setTrajectory(new OneDimensionalTrajectoryRamped(
					tempDist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV),
					MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND),
					MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND), RAMPED_PROFILE_TIME_MULT_ELEVATOR));
		}else {

			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_DOWN, kA_DOWN, kP_DOWN, kD_DOWN);
			basicProfiler.setTrajectory(new OneDimensionalTrajectoryRamped(
					tempDist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV),
					MAX_SPEED_ELEVATOR_DOWN.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND),
					MAX_ACCEL_ELEVATOR_DOWN.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
                            Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND), RAMPED_PROFILE_TIME_MULT_ELEVATOR));
        }

        basicProfiler.enable();
    }
        public void disableProfiler() {
            basicProfiler.disable();
        }
        public void setPIDSourceType(PIDSourceType pidSource) {
            type = pidSource;
        }
        public PIDSourceType getPIDSourceType() {
            return type;
        }
        
        public double pidGet() {
                if(type == PIDSourceType.kRate) {
                    return getInstance().getVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
					    Time.Unit.HUNDRED_MILLISECOND);
                } else{
                        return getInstance().getPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV);
                }
        }
        public void pidWrite(double output) {
            setMotorSpeed(MAX_SPEED_ELEVATOR_UP.mul(output));

        }
        public void smartDashboardInit() {
            if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
                SmartDashboard.putNumber("Elevator Profile Delta Inches: ", 0);
                SmartDashboard.putNumber("Voltage Ramp Rate Elevator Seconds: ",
                        VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND));
    
                SmartDashboard.putNumber("Elevator kA Up: ", kA_UP);
                SmartDashboard.putNumber("Elevator kP Up: ", kP_UP);
                SmartDashboard.putNumber("Elevator kD Up: ", kD_UP);
                SmartDashboard.putNumber("F Pos Elevator Up: ", F_POS_ELEVATOR_UP);
                SmartDashboard.putNumber("P Pos Elevator Up: ", P_POS_ELEVATOR_UP);
                SmartDashboard.putNumber("I Pos Elevator Up: ", I_POS_ELEVATOR_UP);
                SmartDashboard.putNumber("D Pos Elevator Up: ", D_POS_ELEVATOR_UP);
                SmartDashboard.putNumber("P Vel Elevator Up: ", P_VEL_ELEVATOR_UP);
                SmartDashboard.putNumber("I Vel Elevator Up: ", I_VEL_ELEVATOR_UP);
                SmartDashboard.putNumber("D Vel Elevator Up: ", D_VEL_ELEVATOR_UP);
    
                SmartDashboard.putNumber("Elevator kA Down: ", kA_DOWN);
                SmartDashboard.putNumber("Elevator kP Down: ", kP_DOWN);
                SmartDashboard.putNumber("Elevator kD Down: ", kD_DOWN);
                SmartDashboard.putNumber("F Pos Elevator Down: ", F_POS_ELEVATOR_DOWN);
                SmartDashboard.putNumber("P Pos Elevator Down: ", P_POS_ELEVATOR_DOWN);
                SmartDashboard.putNumber("I Pos Elevator Down: ", I_POS_ELEVATOR_DOWN);
                SmartDashboard.putNumber("D Pos Elevator Down: ", D_POS_ELEVATOR_DOWN);
                SmartDashboard.putNumber("P Vel Elevator Down: ", P_VEL_ELEVATOR_DOWN);
                SmartDashboard.putNumber("I Vel Elevator Down: ", I_VEL_ELEVATOR_DOWN);
                SmartDashboard.putNumber("D Vel Elevator Down: ", D_VEL_ELEVATOR_DOWN);
    
                SmartDashboard.putNumber("Profile Vel Percent Elevator: ", PROFILE_VEL_PERCENT_ELEVATOR);
                SmartDashboard.putNumber("Profile Accel Percent Elevator: ", PROFILE_ACCEL_PERCENT_ELEVATOR);
            }
        }

        @Override
        public void smartDashboardInfo() {
            if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_BASIC_ENABLED) {
                SmartDashboard.putNumberArray("ElevatorCurrent: ", new double[] {getMasterCurrent()}); //Put in follower current
                SmartDashboard.putNumberArray("Elevator Velocity vs. Set Velocity: ", new double[] {getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND), velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
                SmartDashboard.putNumberArray("Elevator Position vs. Set Position: ", new double[] {getPosition().get(Distance.Unit.INCH), posSetpoint.get(Distance.Unit.INCH)});
            }             
            if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
            profilePos = new Distance(SmartDashboard.getNumber("Elevator Profile Delta Inches: ", 0), Distance.Unit.INCH);
                F_POS_ELEVATOR_UP = SmartDashboard.getNumber("F Pos Elevator Up: ", F_POS_ELEVATOR_UP);
                elevatorTalon.config_kF(MOTION_MAGIC_ELEV_UP_SLOT, F_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
                P_POS_ELEVATOR_UP = SmartDashboard.getNumber("P Pos Elevator Up: ", P_POS_ELEVATOR_UP);
                elevatorTalon.config_kP(MOTION_MAGIC_ELEV_UP_SLOT, P_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
                I_POS_ELEVATOR_UP = SmartDashboard.getNumber("I Pos Elevator Up: ", I_POS_ELEVATOR_UP);
                elevatorTalon.config_kI(MOTION_MAGIC_ELEV_UP_SLOT, I_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
                D_POS_ELEVATOR_UP = SmartDashboard.getNumber("D Pos Elevator Up: ", D_POS_ELEVATOR_UP);
                elevatorTalon.config_kD(MOTION_MAGIC_ELEV_UP_SLOT, D_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
                P_VEL_ELEVATOR_UP = SmartDashboard.getNumber("P Vel Elevator Up: ", P_VEL_ELEVATOR_UP);
                elevatorTalon.config_kP(VEL_ELEV_UP_SLOT, P_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
                I_VEL_ELEVATOR_UP = SmartDashboard.getNumber("I Vel Elevator Up: ", I_VEL_ELEVATOR_UP);
                elevatorTalon.config_kI(VEL_ELEV_UP_SLOT, I_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
                D_VEL_ELEVATOR_UP = SmartDashboard.getNumber("D Vel Elevator Up: ", D_VEL_ELEVATOR_UP);
                elevatorTalon.config_kD(VEL_ELEV_UP_SLOT, D_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);

                PROFILE_VEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Vel Percent Elevator: ",
                        PROFILE_VEL_PERCENT_ELEVATOR);
                PROFILE_ACCEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Accel Percent Elevator: ",
                        PROFILE_ACCEL_PERCENT_ELEVATOR);
                            
                kA_UP = SmartDashboard.getNumber("Elevator kA Up: ", kA_UP);
                kP_UP = SmartDashboard.getNumber("Elevator kP Up: ", kP_UP);
                kD_UP = SmartDashboard.getNumber("Elevator kD Up: ", kD_UP);
                
                kA_DOWN = SmartDashboard.getNumber("Elevator kA Down: ", kA_DOWN);
                kP_DOWN = SmartDashboard.getNumber("Elevator kP Down: ", kP_DOWN);
                kD_DOWN = SmartDashboard.getNumber("Elevator kD Down: ", kD_DOWN);
                
                SmartDashboard.putNumber("Elevator Encoder Ticks: ", elevatorTalon.getSelectedSensorPosition(PID_TYPE));
            }
        }  
    //replace with Hall Effect Sensor
   // public boolean isRevLimitSwitchClosed() {
     //   return elevatorTalon.getSensorCollection().isRevLimitClosed();
    //}

    @Override
    public void periodic() {
        if (EnabledSubsystems.ELEVATOR_ENABLED) {
            if (OI.getInstance().isKidModeOn()) {
                PROFILE_VEL_PERCENT_ELEVATOR = 0.6;
            } else{ 
                PROFILE_VEL_PERCENT_ELEVATOR = 0.8;
            }
            if (elevatorTalon.getSensorCollection().isRevLimitSwitchClosed()) {
                elevatorTalon.getSensorCollection().setQuadraturePosition((int) CARGO_PICKUP_HEIGHT_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV), DEFAULT_TIMEOUT);
            }
        }
    }

    @Override
    public void disable() {
        setMotorSpeedPercent(0);
    }
    } 