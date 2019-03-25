package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.multicommands.ScoreHatchCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.LineSensorStrafeCommandGroup;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToCargoShipFrontLeftProfilingCommand extends CommandGroup {

    public StartPosMiddleToCargoShipFrontLeftProfilingCommand(){
        //addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "StartPosMiddleToCargoShipFrontLeft"));
    
        addSequential(new EnableMotionProfile(new Distance(11.83, Distance.Unit.FOOT), Distance.ZERO, Drive.ONE_D_PROFILE_DRIVE_PERCENT, Drive.ONE_D_PROFILE_ACCEL_PERCENT));

        addSequential(new LineSensorStrafeCommandGroup(0));

        addSequential(new ScoreHatchCommand());

        
    }
}
