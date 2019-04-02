package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers.State;

public class IntakeRollersDeployToggleCommand extends NRCommand {

	public IntakeRollersDeployToggleCommand() {
		super(IntakeRollers.getInstance());
	}
	
	public void onStart() {
		IntakeRollers.getInstance().disable();

        if (IntakeRollers.getInstance().currentDeployState() == State.RETRACTED)
            IntakeRollers.getInstance().deployIntakeRollers();
        else
            IntakeRollers.getInstance().retractIntakeRollers();
	}
	
}