package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class ReleaseHatchCommand extends NRCommand {

	public ReleaseHatchCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
		HatchMechanism.getInstance().releaseHatch();
	}
	
}