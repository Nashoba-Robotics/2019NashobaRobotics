package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.multicommands.GetHatchStationCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToSomethingCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismGrabCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoShipFrontRightToHatchProfilingCommand extends CommandGroup {

    public CargoShipFrontRightToHatchProfilingCommand() {
        addSequential(new GetHatchStationCommand());

        addSequential(new EnableMotionProfile(new Distance(-6, Distance.Unit.INCH), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

        addSequential(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));

        addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "CargoShipFrontRightToHatch"));

        addSequential(new TurnToSomethingCommand(Pipeline.Target));

        addSequential(new EnableMotionProfile(new Distance(36, Distance.Unit.INCH), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

        addSequential(new HatchMechanismGrabCommand());
        
        
        //grab hatch somehow
    }

}
