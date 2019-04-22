package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.robotics.multicommands.GetHatchStationCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class VerifyHatchGrabbedCommand extends CommandGroup {


    public VerifyHatchGrabbedCommand() {
        addSequential(new WaitCommand(0.5));

        addSequential(new ConditionalCommand(new DoNothingCommand(HatchMechanism.getInstance()), new GetHatchStationCommand()){
        
            @Override
            protected boolean condition() {
                return EnabledSensors.hatchLimitSwitch1.get() && EnabledSensors.hatchLimitSwitch2.get();
            }
        });
    }

}