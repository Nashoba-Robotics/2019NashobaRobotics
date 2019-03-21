package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicCommand;
import edu.nr.robotics.subsystems.lift.Lift;
import edu.nr.robotics.subsystems.lift.LiftSetPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class RetractLiftCommand extends CommandGroup {

    public RetractLiftCommand() {
        addSequential(new ElevatorMoveBasicCommand(new Distance(7, Distance.Unit.INCH), 0.5));

        addSequential(new WaitCommand(0.25));

        addSequential(new LiftSetPositionCommand(Distance.ZERO, Lift.profilePercent));

        
    }

}
