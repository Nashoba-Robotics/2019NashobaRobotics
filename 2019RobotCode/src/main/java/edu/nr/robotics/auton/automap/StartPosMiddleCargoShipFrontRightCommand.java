package edu.nr.robotics.auton.automap;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToCargoShipFrontRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleCargoShipFrontRightCommand extends CommandGroup{

    public StartPosMiddleCargoShipFrontRightCommand() {

        addSequential(new StartPosMiddleToCargoShipFrontRightProfilingCommand());
    }

}
