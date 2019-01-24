package edu.nr.robotics.subsystems.hatchMechanism;

import edu.nr.lib.commandbased.DoNothingJoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchMechanism extends NRSubsystem {
	public static HatchMechanism singleton;

    private DoubleSolenoid deploySolenoid;
    private DoubleSolenoid hatchSolenoid;

	public enum State {
		DEPLOYED, RETRACTED;
		
		private static Value DEPLOYED_VALUE = Value.kForward;
		private static Value RETRACTED_VALUE = Value.kReverse;
		

		
		private static State get(Value val) {
			if(val == State.DEPLOYED_VALUE) {
				return State.DEPLOYED;
			} else {
				return State.RETRACTED;
			}
		}
	}

	public State currentDeployState() {
		if(deploySolenoid != null) {
			return State.get(deploySolenoid.get());
		} else {
			return State.DEPLOYED; //TODO: Should be State.RETRACTED, is deployed for testing
		}
    }
    
    public State currentHatchState() {
        if(hatchSolenoid != null) {
            return State.get(hatchSolenoid.get());
        } else {
            return State.DEPLOYED;
        }
    }


	private HatchMechanism() {
		if (EnabledSubsystems.HATCH_MECHANISM_ENABLED) {
			deploySolenoid = new DoubleSolenoid(RobotMap.HATCH_MECHANISM_DEPLOY_PCM_PORT, RobotMap.HATCH_MECHANISM_DEPLOY_FORWARD,
                    RobotMap.HATCH_MECHANISM_DEPLOY_REVERSE);
            hatchSolenoid = new DoubleSolenoid(RobotMap.HATCH_MECHANISM_HATCH_PCM_PORT, RobotMap.HATCH_MECHANISM_HATCH_FORWARD,
             RobotMap.HATCH_MECHANISM_HATCH_REVERSE);
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
            hatchSolenoid.set(State.DEPLOYED_VALUE);
        }
    }

    void releaseHatch() {
        if (hatchSolenoid != null) {
            hatchSolenoid.set(State.RETRACTED_VALUE);
        }
    }

	@Override
	public void smartDashboardInfo() {
		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED){
            SmartDashboard.putString("Hatch Deploy Mechanism Position: ", currentDeployState().toString());
            SmartDashboard.putString("Hatch Mechanism State: ", currentHatchState().toString());
		}
		if(EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED){
			
		}

	}

	@Override
	public void periodic() {
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
