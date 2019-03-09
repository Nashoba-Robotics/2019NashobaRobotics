package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ReturnToNeutralPositionCommand extends CommandGroup {

    public ReturnToNeutralPositionCommand() {

        addSequential(new IntakeRollersRetractCommand());

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new HatchMechanismRetractCommand());

        addSequential(new ElevatorPositionCommand(Distance.ZERO));

    }

}
