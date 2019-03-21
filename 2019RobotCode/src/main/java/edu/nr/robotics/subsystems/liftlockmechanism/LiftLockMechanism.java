package edu.nr.robotics.subsystems.liftlockmechanism;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftLockMechanism extends NRSubsystem {
    public static LiftLockMechanism singleton;

    private Solenoid lockSolenoid;

    public static final Time ACTUATION_TIME = new Time(0.75, Time.Unit.SECOND);

    public enum State {
        DEPLOYED, RETRACTED;

        private static boolean DEPLOYED_VALUE = false;
		private static boolean RETRACTED_VALUE = true;
		
		private static State getDeployState(boolean val) {
			if(val == State.DEPLOYED_VALUE) {
				return State.DEPLOYED;
			} else {
				return State.RETRACTED;
			}
		}
	}


	public State currentLockState() {
		if(lockSolenoid != null) {
			return State.getDeployState(lockSolenoid.get());
		} else {
			return State.DEPLOYED; //TODO: Should be State.RETRACTED, is deployed for testing
		}
    }

    public LiftLockMechanism() {
        if (EnabledSubsystems.LIFT_LOCK_MECHANISM_ENABLED) {
            lockSolenoid = new Solenoid(RobotMap.PCM_ID, RobotMap.LIFT_LOCK_MECHANISM_PCM_PORT);
        }
        
    }

    public static LiftLockMechanism getInstance() {
        if (singleton == null) {
            init();
        }
        
        return singleton;
    }

    public synchronized static void init() {
		if (singleton == null) {
			singleton = new LiftLockMechanism();
		}
	}

    public void deployLiftLockMechanism() {
		if (lockSolenoid != null) {
			lockSolenoid.set(State.DEPLOYED_VALUE);
		}
	}

	public void retractLiftLockMechanism() {
		if (lockSolenoid != null) {
			lockSolenoid.set(State.RETRACTED_VALUE);
		}
    }

    @Override
    public void smartDashboardInfo() {
        if(EnabledSubsystems.LIFT_LOCK_MECHANISM_SMARTDASHBOARD_BASIC_ENABLED) {
            SmartDashboard.putString("Hatch Deploy Mechanism Position: ", currentLockState().toString());
		}
    }

    @Override
    public void disable() {

    }

    public boolean isLiftLockMechanismDeployed() {
        return (currentLockState() == State.DEPLOYED);
    }

}