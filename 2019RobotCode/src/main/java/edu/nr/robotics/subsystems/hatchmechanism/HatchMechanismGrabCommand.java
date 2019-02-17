package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;

public class HatchMechanismGrabCommand extends NRCommand {

    public HatchMechanismGrabCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onStart() {
        HatchMechanism.getInstance().grabHatch();
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
