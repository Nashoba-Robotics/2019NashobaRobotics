package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorCurrentCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ClimbCommandGroup extends CommandGroup {

    public ClimbCommandGroup(Distance elevHeight) {
        addSequential(new ConditionalCommand(new AnonymousCommandGroup(){

            @Override
            public void commands() {
                new PrepareClimbCommand(elevHeight);

                addSequential(new WaitCommand(0.5));
            }
            
        }) {

            @Override
            protected boolean condition() {
                return Elevator.getInstance().getPosition().lessThan(elevHeight);
            }

        });

        addSequential(new ElevatorCurrentCommand(Elevator.CLIMB_PERCENT, Elevator.CLIMB_CURRENT_SPIKE));

        addSequential(new ClimbCommand());
    }
}