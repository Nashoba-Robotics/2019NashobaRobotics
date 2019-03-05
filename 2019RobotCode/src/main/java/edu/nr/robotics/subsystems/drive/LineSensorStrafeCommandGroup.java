package edu.nr.robotics.subsystems.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class LineSensorStrafeCommandGroup extends CommandGroup {

    public LineSensorStrafeCommandGroup(double drivePercent) {

        addSequential(new LineSensorStrafeLargeCommand(drivePercent));

        addSequential(new LineSensorStrafeCommand());

        addSequential(new ConditionalCommand(new HKickCommand(Drive.KICK_PERCENT, Drive.HKICK_TIME)) {
        
            @Override
            protected boolean condition() {
                return (LineSensorStrafeCommand.tracker == 1);
            }
        });

        addSequential(new ConditionalCommand(new HKickCommand(-Drive.KICK_PERCENT, Drive.HKICK_TIME)) {
        
            @Override
            protected boolean condition() {
                return (LineSensorStrafeCommand.tracker == 2);
            }
        });
    }

}