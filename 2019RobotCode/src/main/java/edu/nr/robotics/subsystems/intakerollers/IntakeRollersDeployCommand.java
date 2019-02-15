package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersDeployCommand extends NRCommand {

	public IntakeRollersDeployCommand() {
		super(IntakeRollers.getInstance());
	}
	
	public void onStart() {
		IntakeRollers.getInstance().deployIntakeRollers();
	}
	
}