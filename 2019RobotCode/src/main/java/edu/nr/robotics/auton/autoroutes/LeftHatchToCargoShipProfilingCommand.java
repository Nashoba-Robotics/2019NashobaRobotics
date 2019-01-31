package edu.nr.robotics.auton.autoroutes;

import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftHatchToCargoShipProfilingCommand extends CommandGroup{

    public LeftHatchToCargoShipProfilingCommand() {
      //  addSequential(new EnableTwoDMotionProfile(xProfile, yProfile, endAngle, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "LeftHatchToCargoShip"));
    }

}
