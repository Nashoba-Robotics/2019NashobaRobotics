package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismGrabCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ReturnToNeutralPositionCommand extends CommandGroup {

    public ReturnToNeutralPositionCommand() {

        addSequential(new IntakeRollersStopCommand());

        addSequential(new LiftLockMechanismRetractCommand());

        addSequential(new IntakeRollersRetractCommand());

        addSequential(new HatchMechanismGrabCommand());

        addSequential(new HatchMechanismRetractCommand());

        addSequential(new ElevatorPositionCommand(Distance.ZERO));

    }

}
