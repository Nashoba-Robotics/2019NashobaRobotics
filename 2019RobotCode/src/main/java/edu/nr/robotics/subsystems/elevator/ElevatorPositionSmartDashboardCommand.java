package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorPositionSmartDashboardCommand extends NRCommand {

	private Distance height;
	
	public ElevatorPositionSmartDashboardCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		this.height = Elevator.profilePos;
		Elevator.getInstance().setPosition(height);
	}

	protected void onEnd() {
		if (!height.equals(Distance.ZERO)) {}
			//new ElevatorHoldPositionCommand().start();
	}
	
	@Override
	protected boolean isFinishedNR() {
		
		boolean finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD)
				&& (Elevator.getInstance().getPosition().sub(height)).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
				
		return finished;
	}
}