
package edu.nr.robotics.auton.autoroutes;

import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftRocketBackToHatchProfilingCommand extends CommandGroup {

    public LeftRocketBackToHatchProfilingCommand() {
       // addSequential(new EnableTwoDMotionProfile(x, y, a, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "LeftRocketBackToHatch"));
    }

}
