package edu.nr.robotics.subsystems.liftlockmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class LiftLockMechanismRetractCommand extends NRCommand {

	public LiftLockMechanismRetractCommand() {
		super(LiftLockMechanism.getInstance());
	}
	
	public void onStart() {
		LiftLockMechanism.getInstance().retractLiftLockMechanism();
	}
	
}