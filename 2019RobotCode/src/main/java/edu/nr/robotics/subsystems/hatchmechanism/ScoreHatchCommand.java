package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScoreHatchCommand extends CommandGroup {

    public ScoreHatchCommand() {
        addSequential(new HatchMechanismDeployCommand());

        //addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new HatchMechanismReleaseCommand());

        addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new HatchMechanismRetractCommand());
    }

}