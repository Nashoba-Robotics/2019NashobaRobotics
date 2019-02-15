package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism.State;

public class DeployHatchToggleCommand extends NRCommand {

	public DeployHatchToggleCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
        if (HatchMechanism.getInstance().currentDeployState() == State.DEPLOYED)
            HatchMechanism.getInstance().retractHatchMechanism();
        else
            HatchMechanism.getInstance().deployHatchMechanism();
	}
	
}