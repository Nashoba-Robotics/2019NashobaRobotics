package edu.nr.robotics.auton.automap;
import edu.nr.robotics.auton.autoroutes.StartPosRightToShipSideProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightCargoShipSideCommand extends CommandGroup{

    public StartPosRightCargoShipSideCommand() {
        addSequential(new StartPosRightToShipSideProfilingCommand());
    }

}
