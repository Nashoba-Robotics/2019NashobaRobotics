package edu.nr.robotics.auton.automap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToRocketFrontProfilingCommand;
public class StartPosLeftRocketFrontCommand extends CommandGroup {

    public StartPosLeftRocketFrontCommand() {
        addSequential(new StartPosLeftToRocketFrontProfilingCommand());
    }

}
