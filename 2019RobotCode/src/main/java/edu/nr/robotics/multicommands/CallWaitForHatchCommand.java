package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism;
import edu.nr.robotics.subsystems.hatchmechanism.WaitForHatchCommand;

public class CallWaitForHatchCommand extends NRCommand {

    public CallWaitForHatchCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onEnd() {
        new WaitForHatchCommand().start();
    }

    protected boolean isFinnishedNR() {
        return true;
    }
}
