package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class GrabHatchFromStationCommand extends CommandGroup {

    public GrabHatchFromStationCommand() {

        /*addSequential(new ConditionalCommand(new AnonymousCommandGroup(){
        
            @Override
            public void commands() {
                addSequential(new HatchMechanismReleaseCommand());

                addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));
                
            }
        }) {

            @Override
            protected boolean condition() {
                return HatchMechanism.getInstance().isHatchGrabDeployed() == true;
            }

        });*/

        //addSequential(new HatchMechanismDeployCommand());

        //addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new HatchMechanismGrabCommand());

        //addSequential(new WaitCommand(HatchMechanism.ACTUATION_TIME.get(Time.Unit.SECOND)));

        addSequential(new HatchMechanismRetractCommand());

    }
}
