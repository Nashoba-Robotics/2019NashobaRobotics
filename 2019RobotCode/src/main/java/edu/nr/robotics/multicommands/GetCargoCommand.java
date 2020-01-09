package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetCargoCommand extends CommandGroup {

    public GetCargoCommand() {
        addSequential(new ElevatorPositionCommand(Elevator.CARGO_PICKUP_HEIGHT_ELEVATOR));

        addSequential(new HatchMechanismRetractCommand());

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new IntakeRollersDeployCommand());

        //addParallel(new ElevatorPercentRawCommand(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP));

        addSequential(new IntakeRollersIntakeCommand());
    }
}
