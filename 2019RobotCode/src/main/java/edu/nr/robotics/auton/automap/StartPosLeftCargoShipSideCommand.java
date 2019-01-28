package edu.nr.robotics.auton.automap;

import edu.nr.robotics.auton.autoroutes.StartPosLeftToCargoShipSideProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftCargoShipSideCommand extends CommandGroup{

    public StartPosLeftCargoShipSideCommand() {
        addSequential(new StartPosLeftToCargoShipSideProfilingCommand());
    }

}