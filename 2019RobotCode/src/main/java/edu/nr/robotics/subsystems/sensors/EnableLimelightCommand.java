package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;

public class EnableLimelightCommand extends NRCommand {

	private boolean boo;

	public EnableLimelightCommand(boolean boo) {

		this.boo = boo;

	}	

	protected void onStart() {
		EnabledSensors.limelightEnabled = boo;

		if (boo) {
			LimelightNetworkTable.getInstance().lightLED(true);
			LimelightNetworkTable.getInstance().enable();
		} else {
			LimelightNetworkTable.getInstance().lightLED(false);
			LimelightNetworkTable.getInstance().disable();
		}

	}

	protected boolean isFinishedNR() {
		return true;
	}
}