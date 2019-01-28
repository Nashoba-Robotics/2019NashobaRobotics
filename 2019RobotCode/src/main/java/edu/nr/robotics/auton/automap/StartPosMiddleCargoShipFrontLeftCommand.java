package edu.nr.robotics.auton.automap;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToCargoShipFrontLeftProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleCargoShipFrontLeftCommand extends CommandGroup{

    public StartPosMiddleCargoShipFrontLeftCommand() {

        addSequential(new StartPosMiddleToCargoShipFrontLeftProfilingCommand());
    }

}
