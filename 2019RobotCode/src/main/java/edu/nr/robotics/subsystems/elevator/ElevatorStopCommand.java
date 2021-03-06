package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorStopCommand extends NRCommand {

	public ElevatorStopCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().disable();
	}
	
	@Override 
	protected boolean isFinishedNR() {
		return true;
	}
}