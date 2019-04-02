package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.nr.robotics.subsystems.hatchmechanism.WaitForHatchCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetHatchStationCommand extends CommandGroup {

    public GetHatchStationCommand() {
        addSequential(new ElevatorPositionCommand(Elevator.HATCH_PLACE_LOW_HEIGHT_ELEVATOR));
      
        addSequential(new IntakeRollersRetractCommand());

        addSequential(new HatchMechanismRetractCommand());

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new WaitForHatchCommand());
    }
}