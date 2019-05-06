package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.DoNothingJoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Enabled;

public class HatchMechanism extends NRSubsystem {
	public static HatchMechanism singleton;

    private Solenoid deploySolenoid;
	private Solenoid hatchSolenoid;

	public boolean forceSensors = false;
	public boolean prevForceSensors = false;

	public static final Time ACTUATION_TIME = new Time(0.5, Time.Unit.SECOND);

	public static int hatchSensorThreshold = 1160;

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
			hatchSolenoid = new Solenoid(RobotMap.PCM_ID ,RobotMap.HATCH_MECHANISM_HATCH_PCM_PORT);
			
			SmartDashboardInit();
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
			System.out.println("102");
            hatchSolenoid.set(State.DEPLOYED_VALUE_HATCH);
        }
    }

    void releaseHatch() {
        if (hatchSolenoid != null) {
			System.out.println("109");
            hatchSolenoid.set(State.RETRACTED_VALUE_HATCH);
        }
	}
	

	public void SmartDashboardInit() {
		if (EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Hatch Sensor Threshold: ", hatchSensorThreshold);
		}
	}

	@Override
	public void smartDashboardInfo() {
		SmartDashboard.putBoolean("Has Hatch", hasHatch());

		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED) {
            SmartDashboard.putString("Hatch Deploy Mechanism Position: ", currentDeployState().toString());
            SmartDashboard.putString("Hatch Mechanism State: ", currentHatchState().toString());
		}
		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_DEBUG_ENABLED) {
			hatchSensorThreshold = (int) SmartDashboard.getNumber("Hatch Sensor Threshold: ", hatchSensorThreshold);
			EnabledSensors.hatchSensor1.setThreshold(hatchSensorThreshold);
			EnabledSensors.hatchSensor2.setThreshold(hatchSensorThreshold);
		}

	}

	@Override
	public void periodic() {

	}

	@Override
	public void disable() {
	}

	public boolean hasHatch() {
		return EnabledSensors.hatchSensor1.get() && EnabledSensors.hatchSensor2.get();
	}

	public boolean isHatchMechanismDeployed() {
		return currentDeployState() == State.DEPLOYED;
    }
    
    public boolean isHatchGrabDeployed() {
        return currentHatchState() == State.DEPLOYED;
    }

}
