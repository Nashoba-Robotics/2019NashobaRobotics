package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
public class GetCargoCommand extends CommandGroup {


    public GetCargoCommand() {
        addSequential(new ElevatorPositionCommand(Distance.ZERO));

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new IntakeRollersDeployCommand());

        //addParallel(new ElevatorPercentRawCommand(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP));

        addSequential(new IntakeRollersIntakeCommand());
    }
}
