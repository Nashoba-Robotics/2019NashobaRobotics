package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class LineSensorLoopCommand extends NRCommand {

    public LineSensorLoopCommand() {
        super(Drive.getInstance());
    }

    protected void onEnd() {
        new LineSensorStrafeCommandGroup(0).start();
    }

    protected boolean isFinishedNR() {
        return true;
    }
}