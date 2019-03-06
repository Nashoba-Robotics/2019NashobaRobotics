package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class WaitForHatchCommand extends NRCommand {

    public WaitForHatchCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onEnd() {
        HatchMechanism.getInstance().grabHatch();
    }

    protected boolean isFinishedNR() {
        return (!EnabledSensors.hatchForceSensorOne.get() && !EnabledSensors.hatchForceSensorOne.get());
    }

}