
package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismReleaseCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersRetractCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersVelocityCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;


public class IntakeToggleCommand extends CommandGroup { 
/*
    public IntakeToggleCommand() {
        addSequential(new ConditionalCommand(new AnonymousCommandGroup() {
        
            public void commands() {
                addSequential(new HatchMechanismReleaseCommand());

                addSequential(new GetCargoCommand());
            }

        }) {

            @Override
            protected boolean condition() {
                return !IntakeRollers.getInstance().isIntakeRollersDeployed() && !HatchMechanism.getInstance().hasHatch();
            }

        });

        addSequential(new ConditionalCommand(new AnonymousCommandGroup() {
        
            @Override
            public void commands() {

                addSequential(new IntakeRollersVelocityCommand(0));

                addSequential(new IntakeRollersRetractCommand());
                
            }
        }){
        
            @Override
            protected boolean condition() {
                return IntakeRollers.getInstance().isIntakeRollersDeployed() && !IntakeRollers.getInstance().hasCargo();
            }
        });
        
           

    }*/
}
