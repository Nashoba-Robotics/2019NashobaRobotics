package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersWaitForBallCommand extends NRCommand {

    double time = 0;
	double spiketime = 0;

    @Override
	public void onEnd() {
		//IntakeRollers.getInstance().disable();
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().HOLD_PERCENT);
	}
	
	@Override
	public boolean isFinishedNR() {
		boolean finished = false;

		if (IntakeRollers.getInstance().getCurrent() > 40) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.15) 
				finished = true;
		}
		else {
			time = Timer.getFPGATimestamp();
		}

		return finished;
	}

}
