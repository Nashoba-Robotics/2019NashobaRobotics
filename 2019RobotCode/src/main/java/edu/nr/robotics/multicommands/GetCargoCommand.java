package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetCargoCommand extends CommandGroup {

    public GetCargoCommand() {
        addSequential(new ElevatorPositionCommand(Elevator.CARGO_PICKUP_HEIGHT_ELEVATOR));

        addSequential(new IntakeRollersDeployCommand());

        addSequential(new IntakeRollersIntakeCommand());
    }
}
