package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;

public class LineSensorStrafeCommandGroup extends CommandGroup {

    public LineSensorStrafeCommandGroup(double drivePercent) {

        addSequential(new LineSensorStrafeLargeCommand(drivePercent));

        addSequential(new LineSensorStrafeCommand());

        addSequential(new ConditionalCommand(new AnonymousCommandGroup(){
        
            @Override
            public void commands() {

                addSequential(new ConditionalCommand(new HKickCommand(Drive.KICK_LOW_PERCENT, Drive.HKICK_TIME)) {
        
                    @Override
                    protected boolean condition() {
                        return (LineSensorStrafeCommand.tracker == 1) && LineSensorStrafeLargeCommand.sensorStart;
                    }
                });

                addSequential(new ConditionalCommand(new HKickCommand(Drive.KICK_HIGH_PERCENT, Drive.HKICK_TIME)) {
        
                    @Override
                    protected boolean condition() {
                        return (LineSensorStrafeCommand.tracker == 1) && !LineSensorStrafeLargeCommand.sensorStart;
                    }
                });
        
                addSequential(new ConditionalCommand(new HKickCommand(-Drive.KICK_LOW_PERCENT, Drive.HKICK_TIME)) {
                
                    @Override
                    protected boolean condition() {
                        return (LineSensorStrafeCommand.tracker == 2) && LineSensorStrafeLargeCommand.sensorStart;
                    }
                });
        
                addSequential(new ConditionalCommand(new HKickCommand(-Drive.KICK_HIGH_PERCENT, Drive.HKICK_TIME)) {
                
                    @Override
                    protected boolean condition() {
                        return (LineSensorStrafeCommand.tracker == 2) && !LineSensorStrafeLargeCommand.sensorStart;
                    }
                });
                
            }

        }) {
        
            @Override
            protected boolean condition() {
                return !LineSensorStrafeCommand.sensorStart;
            }
        });

        addSequential(new ConditionalCommand(new LineSensorLoopCommand()) {

            @Override
            protected boolean condition() {
                return new SensorVoting(EnabledSensors.floorSensorTwo.get(), !EnabledSensors.floorSensorThree.get(), EnabledSensors.floorSensorFour.get()).isTrue();
            }

        });

    }

}