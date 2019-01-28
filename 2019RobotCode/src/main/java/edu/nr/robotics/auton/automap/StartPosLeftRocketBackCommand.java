package edu.nr.robotics.auton.automap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToRocketBackProfilingCommand;

public class StartPosLeftRocketBackCommand extends CommandGroup {

    public StartPosLeftRocketBackCommand() {
        addSequential(new StartPosLeftToRocketBackProfilingCommand());
    }

}
