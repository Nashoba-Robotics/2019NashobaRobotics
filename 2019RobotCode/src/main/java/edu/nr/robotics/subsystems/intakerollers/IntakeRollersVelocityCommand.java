package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersVelocityCommand extends NRCommand {

	double time = 0;
	double spiketime = 0;

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
		/*boolean finished = false;

		if (IntakeRollers.getInstance().getCurrent() > 40) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.15) 
				finished = true;
		}
		else {
			time = Timer.getFPGATimestamp();
		}

		return finished;*/
		return true;
	}
}
