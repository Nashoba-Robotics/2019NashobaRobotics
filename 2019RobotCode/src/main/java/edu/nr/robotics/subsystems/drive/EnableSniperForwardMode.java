package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class EnableSniperForwardMode extends NRCommand {

	private boolean boo;
	
	public static final double MOVE_JOYSTICK_MULTIPLIER_LOW = 0.5;
	public static final double MOVE_JOYSTICK_MULTIPLIER_HIGH = 1.0;

	public EnableSniperForwardMode(boolean boo) {
		this.boo = boo;
	}

	protected void onStart() {
		if (boo) {
			Drive.MOVE_JOYSTICK_MULTIPLIER = MOVE_JOYSTICK_MULTIPLIER_LOW;
		} else {
			Drive.MOVE_JOYSTICK_MULTIPLIER = MOVE_JOYSTICK_MULTIPLIER_HIGH;
		}
	}
}