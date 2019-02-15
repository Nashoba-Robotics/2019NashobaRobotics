package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class GetHatchStationCommand extends CommandGroup {

    public GetHatchStationCommand() {
        addSequential(new ElevatorBottomCommand());
      
        addSequential(new IntakeRollersRetractCommand());
    }
}