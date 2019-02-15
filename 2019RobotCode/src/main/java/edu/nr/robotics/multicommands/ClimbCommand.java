package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbCommand extends CommandGroup {

    public ClimbCommand() {
        addSequential(new DeployLiftCommand(Elevator.getInstance().REST_HEIGHT_ELEVATOR, Elevator.getInstance().getPosition()));
    }

}
