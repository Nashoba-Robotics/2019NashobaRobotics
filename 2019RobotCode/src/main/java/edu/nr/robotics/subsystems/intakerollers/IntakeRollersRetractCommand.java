package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersRetractCommand extends NRCommand {

	public IntakeRollersRetractCommand() {
		super(IntakeRollers.getInstance());
	}
	
	public void onStart() {
		IntakeRollers.getInstance().disable();

		IntakeRollers.getInstance().retractIntakeRollers();
	}
	
}