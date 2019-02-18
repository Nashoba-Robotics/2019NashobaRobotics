package edu.nr.robotics;

import edu.nr.lib.commandbased.DoNothingJoystickCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearSwitcher extends NRSubsystem {
	public static GearSwitcher singleton;

	private DoubleSolenoid solenoid;

	public static enum Gear {
		high, low;

		// TODO: Drive: Find which gear directions are forward/reverse
		private static Value HIGH_VALUE = Value.kForward;
		private static Value LOW_VALUE = Value.kReverse;

		private static int HIGH_PROFILE = 1;
        private static int LOW_PROFILE = 0;
	}


	private GearSwitcher() {
			solenoid = new DoubleSolenoid(0, 1);
	}

	public static GearSwitcher getInstance() {
		if(singleton == null) {
			init();
		}
		return singleton;
	}

	public synchronized static void init() {
		if (singleton == null) {
			singleton = new GearSwitcher();
		}
	}

	
	public void switchToHighGear() {
		if (solenoid != null) {
			solenoid.set(Gear.HIGH_VALUE);
		}
	}

	public void switchToLowGear() {
		if (solenoid != null) {
			solenoid.set(Gear.LOW_VALUE);
		}
	}

	public Gear getCurrentGear() {
		if (solenoid != null) {
			if (solenoid.get() == Gear.HIGH_VALUE) {
				return Gear.high;
			} else {
				return Gear.low;
			}
		} else {
			return Gear.low;
		}
	}

	public void switchGear() {
		if (getCurrentGear() == Gear.low) {
			switchToHighGear();
		} else {
			switchToLowGear();
		}
	}

	@Override
	public void smartDashboardInfo() {
	}

	@Override
	public void periodic() {
	}

	@Override
	public void disable() {
	}

}