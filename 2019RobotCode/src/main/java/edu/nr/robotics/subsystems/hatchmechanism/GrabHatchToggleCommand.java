package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism.State;

public class GrabHatchToggleCommand extends NRCommand {

	public GrabHatchToggleCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
        if (HatchMechanism.getInstance().currentHatchState() == State.DEPLOYED)
            HatchMechanism.getInstance().releaseHatch();
        else
            HatchMechanism.getInstance().grabHatch();
	}
	
}