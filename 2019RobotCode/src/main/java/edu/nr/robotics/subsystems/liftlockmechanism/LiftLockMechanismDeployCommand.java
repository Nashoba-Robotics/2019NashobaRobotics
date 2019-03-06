package edu.nr.robotics.subsystems.liftlockmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class LiftLockMechanismDeployCommand extends NRCommand {

	public LiftLockMechanismDeployCommand() {
		super(LiftLockMechanism.getInstance());
	}
	
	public void onStart() {
		LiftLockMechanism.getInstance().deployLiftLockMechanism();
	}
	
}