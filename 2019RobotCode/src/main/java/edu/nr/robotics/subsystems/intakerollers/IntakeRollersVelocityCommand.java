package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class IntakeRollersVelocityCommand extends NRCommand {

	private double percent;
	
	public IntakeRollersVelocityCommand(double percent) {
		super(IntakeRollers.getInstance());
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorPercent(percent);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
