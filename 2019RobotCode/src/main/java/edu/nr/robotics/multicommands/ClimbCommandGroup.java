package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ClimbCommandGroup extends CommandGroup {

    public ClimbCommandGroup(Distance elevHeight) {
        addSequential(new PrepareClimbCommand(elevHeight));

        addSequential(new WaitCommand(0.5));

        addSequential(new ClimbCommand());
    }
}
