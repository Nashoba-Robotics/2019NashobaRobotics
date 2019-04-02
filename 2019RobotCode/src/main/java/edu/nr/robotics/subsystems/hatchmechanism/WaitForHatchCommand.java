package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Timer;

public class WaitForHatchCommand extends NRCommand {

    private double seenTime;
    private double time;

    public WaitForHatchCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onEnd() {
        HatchMechanism.getInstance().grabHatch();
    }

    protected boolean isFinishedNR() {
        boolean finished = false;

		if (!EnabledSensors.hatchSensor1.get() && !EnabledSensors.hatchSensor2.get()) {
			seenTime = Timer.getFPGATimestamp();
			if ((seenTime - time) > 0.05) 
				finished = true;
		}
		else {
			time = Timer.getFPGATimestamp();
		}

		return finished;
    }

}