package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetCargoCommand extends CommandGroup {

    public GetCargoCommand() {
        addSequential(new ElevatorPositionCommand(Distance.ZERO));

        addSequential(new IntakeRollersDeployCommand());

        addSequential(new IntakeRollersIntakeCommand());
    }
}
