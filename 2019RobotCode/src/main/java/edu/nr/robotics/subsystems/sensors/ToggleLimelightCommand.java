package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.commandbased.NRCommand;

public class ToggleLimelightCommand extends NRCommand {

    public ToggleLimelightCommand() {

    }

    protected void onStart() {
        if(LimelightNetworkTable.getInstance().getLED() == 0) {
            LimelightNetworkTable.getInstance().lightLED(false);
        } else {
            LimelightNetworkTable.getInstance().lightLED(true);
        }
    }

    protected boolean isFinishedNR() {
        return true;
    }

}