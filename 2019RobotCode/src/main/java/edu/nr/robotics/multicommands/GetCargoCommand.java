package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetCargoCommand extends CommandGroup {

    public GetCargoCommand() {
        addSequential(new ElevatorBottomCommand());

        addSequential(new IntakeRollersDeployCommand());

        addSequential(new IntakeRollersIntakeCommand());
    }
}
