package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class HatchMechanismDeployCommand extends NRCommand {

	public HatchMechanismDeployCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
		HatchMechanism.getInstance().deployHatchMechanism();
	}
	
}