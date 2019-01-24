package edu.nr.robotics.subsystems.hatchMechanism;

import edu.nr.lib.commandbased.NRCommand;

public class GrabHatchCommand extends NRCommand {

	public GrabHatchCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
		HatchMechanism.getInstance().grabHatch();
	}
	
}