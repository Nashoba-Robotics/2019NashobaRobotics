package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorPositionCommand extends NRCommand {

	private Distance height;
	boolean finished = false;
	double time = 0;
	double stoptime = 0;
	private Distance initialHeight;

	
	public ElevatorPositionCommand(Distance height) {
		super(Elevator.getInstance());
		this.height = height;
	}
	
	@Override
	protected void onStart() {
		initialHeight = Elevator.getInstance().getPosition();
		Elevator.getInstance().setPosition(height);
	}
	
	@Override
	protected boolean isFinishedNR() {

		if (height.sub(initialHeight).abs().lessThan(new Distance(10, Distance.Unit.INCH))) {
			stoptime = Timer.getFPGATimestamp();
			if ((stoptime - time) < 0.5) {
				finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD) && (Elevator.getInstance().getPosition().sub(height)).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR.mul(2));
				System.out.println("finished: " + finished);
			} else
				time = Timer.getFPGATimestamp();
		} else {
			finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD)  
			&& (Elevator.getInstance().getPosition().sub(height)).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
		}
		
		return finished;
	}
}