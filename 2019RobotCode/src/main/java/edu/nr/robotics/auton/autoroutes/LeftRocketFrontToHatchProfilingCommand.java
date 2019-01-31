

package edu.nr.robotics.auton.autoroutes;

import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftRocketFrontToHatchProfilingCommand extends CommandGroup{

    public LeftRocketFrontToHatchProfilingCommand() {
     //   addSequential(new EnableTwoDMotionProfile(xProfile, yProfile, endAngle, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "LeftRocketFrontToHatch"));
    }

}
