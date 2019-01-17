package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.LEDController;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Path;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class FlashLEDsAction implements Action {


	public FlashLEDsAction() {
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
		// Nothing done here, controller updates in mEnabedLooper in robot
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {

		LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);

	}
}