package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToSomethingCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanismGrabCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoShipFrontLeftToHatchProfilingCommand extends CommandGroup {

    public CargoShipFrontLeftToHatchProfilingCommand() {
        addSequential(new EnableMotionProfile(new Distance(-24, Distance.Unit.INCH), Distance.ZERO, Drive.ONE_D_PROFILE_DRIVE_PERCENT, Drive.ONE_D_PROFILE_ACCEL_PERCENT));

        addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));

        addSequential(new EnableTwoDMotionProfile(new Distance(9, Distance.Unit.FOOT), new Distance(11, Distance.Unit.FOOT), new Angle(-90, Angle.Unit.DEGREE), new Distance(1, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), Distance.ZERO, Angle.ZERO, Drive.TWO_D_PROFILE_DRIVE_PERCENT, Drive.TWO_D_ACCEL_PERCENT, "CargoShipFrontLeftToHatch"));

        addSequential(new TurnToSomethingCommand(Pipeline.Target));

        addSequential(new EnableMotionProfile(new Distance(46, Distance.Unit.INCH), Distance.ZERO, Drive.ONE_D_PROFILE_DRIVE_PERCENT, Drive.ONE_D_PROFILE_ACCEL_PERCENT));

        addSequential(new HatchMechanismGrabCommand());


        });

        //get hatch
    }

}
