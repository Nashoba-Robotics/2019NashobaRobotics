package edu.nr.robotics.auton.autoroutes;

import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftRocketFrontToCargoProfilingCommand extends CommandGroup{

    public LeftRocketFrontToCargoProfilingCommand() {
      //  addSequential(new EnableTwoDMotionProfile(xProfile, yProfile, endAngle, Drive.PROFILE_DRIVE_PERCENT, Drive.accelPercent, "LeftRocketFrontToCargo"));
    }

}
