package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorCurrentCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorSwitchToClimbGearCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanism;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismDeployCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PrepareClimbCommand extends CommandGroup {

    public PrepareClimbCommand(Distance elevHeight) {

        addSequential(new ElevatorPositionCommand(elevHeight));

        addSequential(new LiftLockMechanismRetractCommand());

        addSequential(new WaitCommand(LiftLockMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new LiftLockMechanismDeployCommand());

        addSequential(new ElevatorSwitchToClimbGearCommand());

    }
 
}
