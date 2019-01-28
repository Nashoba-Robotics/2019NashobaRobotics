package edu.nr.robotics.auton.automap;
import edu.nr.robotics.auton.autoroutes.StartPosRightToRocketFrontProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightRocketFrontCommand extends CommandGroup{

public StartPosRightRocketFrontCommand() {
    addSequential(new StartPosRightToRocketFrontProfilingCommand());
}

}