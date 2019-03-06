package edu.nr.robotics.subsystems.liftlockmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanism.State;

public class LiftLockMechanismToggleCommand extends NRCommand {

	public LiftLockMechanismToggleCommand() {
		super(LiftLockMechanism.getInstance());
	}
	
	public void onStart() {
        if (LiftLockMechanism.getInstance().currentLockState() == State.DEPLOYED)
            LiftLockMechanism.getInstance().retractLiftLockMechanism();
        else if (LiftLockMechanism.getInstance().currentLockState() == State.RETRACTED)
            LiftLockMechanism.getInstance().deployLiftLockMechanism();
	}
	
}