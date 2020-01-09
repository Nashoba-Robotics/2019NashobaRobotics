package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPercentRawCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismDeployCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScoreHatchCommand extends CommandGroup {
    
    public ScoreHatchCommand() {

        //addParallel(new ElevatorPercentRawCommand(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP));

        addParallel(new ConditionalCommand(new ElevatorPercentRawCommand(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP)) {

            @Override
            protected boolean condition() {
                return !(Elevator.getInstance().getPosition().lessThan(new Distance(4, Distance.Unit.INCH)));
            }

        });

        addSequential(new AnonymousCommandGroup(){
        
            @Override
            public void commands() {
                addSequential(new HatchMechanismDeployCommand());
        
                addSequential(new HatchMechanismReleaseCommand());
        
                addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

                addSequential(new HatchMechanismRetractCommand());

                addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));
                
            }
        });

        //addSequential(new ElevatorPositionCommand(Distance.ZERO));

        addSequential(new GetHatchStationCommand());
    }

}