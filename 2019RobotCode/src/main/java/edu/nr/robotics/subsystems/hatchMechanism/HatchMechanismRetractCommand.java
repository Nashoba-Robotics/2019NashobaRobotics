package edu.nr.robotics.subsystems.hatchMechanism;

import edu.nr.lib.commandbased.NRCommand;

public class HatchMechanismRetractCommand extends NRCommand {

	public HatchMechanismRetractCommand() {
		super(HatchMechanism.getInstance());
	}
	
	public void onStart() {
		HatchMechanism.getInstance().retractHatchMechanism();
	}
	
}