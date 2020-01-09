package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployToggleCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class ToggleIntakeCommand extends CommandGroup { 

    public ToggleIntakeCommand() {
        addSequential(new HatchMechanismRetractCommand());

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new IntakeRollersDeployToggleCommand());

    }
}
