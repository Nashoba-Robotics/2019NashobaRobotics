package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class GrabHatchFromStationCommand extends CommandGroup {

    public GrabHatchFromStationCommand() {
        addSequential(new HatchMechanismGrabCommand());

        addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new HatchMechanismRetractCommand());
    }

}
