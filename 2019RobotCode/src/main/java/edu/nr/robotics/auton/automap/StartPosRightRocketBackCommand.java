package edu.nr.robotics.auton.automap;
import edu.nr.robotics.auton.autoroutes.StartPosRightToRocketBackProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightRocketBackCommand extends CommandGroup{

    public StartPosRightRocketBackCommand() {
        addSequential(new StartPosRightToRocketBackProfilingCommand());
    }

}
