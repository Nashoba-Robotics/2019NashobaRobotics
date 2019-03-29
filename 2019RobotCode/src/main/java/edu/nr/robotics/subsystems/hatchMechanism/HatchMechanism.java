package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.DoNothingJoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.multicommands.ScoreHatchCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchMechanism extends NRSubsystem {
	public static HatchMechanism singleton;

    private Solenoid deploySolenoid;
	private Solenoid hatchSolenoid;

	public boolean forceSensors = false;
	public boolean prevForceSensors = false;

	public static final Time ACTUATION_TIME = new Time(0.35, Time.Unit.SECOND);

	public static final int HATCH_SENSOR_THRESHOLD = 900;

	int i = 1;

	public enum State {
		DEPLOYED, RETRACTED;
		
		private static boolean DEPLOYED_VALUE = true;
		private static boolean RETRACTED_VALUE = false;
		private static boolean DEPLOYED_VALUE_HATCH = false;
		private static boolean RETRACTED_VALUE_HATCH = true;
		
		private static State getDeployState(boolean val) {
			if(val == State.DEPLOYED_VALUE) {
				return State.DEPLOYED;
			} else {
				return State.RETRACTED;
			}
		}

		private static State getHatchState(boolean val) {
			if(val == State.DEPLOYED_VALUE_HATCH) {
				return State.DEPLOYED;
			} else {
				return State.RETRACTED;
			}
		}
	}

	public State currentDeployState() {
		if(deploySolenoid != null) {
			//System.out.println(deploySolenoid.get());
			return State.getDeployState(deploySolenoid.get());
		} else {
			return State.DEPLOYED; //TODO: Should be State.RETRACTED, is deployed for testing
		}
    }
    
    public State currentHatchState() {
        if(hatchSolenoid != null) {
            return State.getHatchState(hatchSolenoid.get());
        } else {
            return State.DEPLOYED;
        }
    }

	private HatchMechanism() {
		if (EnabledSubsystems.HATCH_MECHANISM_ENABLED) {
			deploySolenoid = new Solenoid(RobotMap.PCM_ID, RobotMap.HATCH_MECHANISM_DEPLOY_PCM_PORT);
			//deploySolenoid.set(false);
            hatchSolenoid = new Solenoid(RobotMap.PCM_ID, RobotMap.HATCH_MECHANISM_HATCH_PCM_PORT);
			// hatchSolenoid.set(Value.kForward);

		}
	}

	public static HatchMechanism getInstance() {
		if(singleton == null) {
			init();
		}
		return singleton;
	}

	public synchronized static void init() {
		if (singleton == null) {
			singleton = new HatchMechanism();
			singleton.setJoystickCommand(new DoNothingJoystickCommand(singleton));
		}
	}

	void deployHatchMechanism() {
		if (deploySolenoid != null) {
			deploySolenoid.set(State.DEPLOYED_VALUE);
		}
	}

	void retractHatchMechanism() {
		if (deploySolenoid != null) {
			deploySolenoid.set(State.RETRACTED_VALUE);
		}
    }
    
    void grabHatch() {
        if (hatchSolenoid != null) {
			hatchSolenoid.set(State.DEPLOYED_VALUE_HATCH);
        }
    }

    void releaseHatch() {
        if (hatchSolenoid != null) {
            hatchSolenoid.set(State.RETRACTED_VALUE_HATCH);
        }
    }

	@Override
	public void smartDashboardInfo() {
		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED){
            SmartDashboard.putString("Hatch Deploy Mechanism Position: ", currentDeployState().toString());
            SmartDashboard.putString("Hatch Mechanism State: ", currentHatchState().toString());
		}
		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED){
			SmartDashboard.putData(new ScoreHatchCommand()); // change NICK
		}

	}

	@Override
	public void periodic() {
		if (i % 25 == 0) {
			i = 1;
			System.out.println("Hatch Sensor 1: " + EnabledSensors.hatchSensor1.getSensor().getValue());
			System.out.println("Hatch Sensor 2: " + EnabledSensors.hatchSensor2.getSensor().getValue());
			//System.out.println("Hatch Sensor 2: " + EnabledSensors.hatchSensor1.get());
			//System.out.println("Hatch Sensor 1: " + EnabledSensors.hatchSensor2.get());
		}

		i++;
	}

	@Override
	public void disable() {
	}

	public boolean isHatchMechanismDeployed() {
		return currentDeployState() == State.DEPLOYED;
    }
    
    public boolean isHatchGrabDeployed() {
        return currentHatchState() == State.DEPLOYED;
    }

}
