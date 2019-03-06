package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorHatchPickupCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetHatchFloorCommand extends CommandGroup {

    public GetHatchFloorCommand() {
        addSequential(new ElevatorPositionCommand(Distance.ZERO));

        addParallel(new ElevatorHatchPickupCommand());
      
        addSequential(new IntakeRollersDeployCommand());

        addSequential(new IntakeRollersRetractCommand());
    }
}