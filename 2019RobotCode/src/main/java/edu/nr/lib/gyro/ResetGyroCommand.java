package edu.nr.lib.gyro;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;

public class ResetGyroCommand extends NRCommand {

	public ResetGyroCommand() {
		
	}

	@Override
	protected void onStart() {
		Pigeon.getPigeon(RobotMap.PIGEON_ID).reset();
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
