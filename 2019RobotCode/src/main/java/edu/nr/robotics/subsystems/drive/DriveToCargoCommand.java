package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;


public class DriveToCargoCommand extends NRCommand {

    public DriveToCargoCommand() {
        super(Drive.getInstance());
    }

    protected void onStart() {
        new EnableLimelightCommand(true).start();
    }


}