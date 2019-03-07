package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;

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
