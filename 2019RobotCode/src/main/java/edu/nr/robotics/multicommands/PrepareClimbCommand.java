package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareClimbCommand extends CommandGroup {

    public PrepareClimbCommand(Distance elevHeight) {
        addSequential(new ElevatorPositionCommand(elevHeight));

        addSequential(new IntakeRollersDeployCommand());
    }
 
}
