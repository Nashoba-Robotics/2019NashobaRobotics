package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class HatchMechanismReleaseCommand extends NRCommand {

    public HatchMechanismReleaseCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onStart() {
        HatchMechanism.getInstance().releaseHatch();
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
