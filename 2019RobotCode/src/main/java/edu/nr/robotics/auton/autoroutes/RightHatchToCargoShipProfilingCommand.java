package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.EnableReverseTwoDMotionProfile;
import edu.nr.robotics.subsystems.drive.LineSensorStrafeCommandGroup;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.hatchmechanism.ScoreHatchCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class RightHatchToCargoShipProfilingCommand extends CommandGroup {

    public RightHatchToCargoShipProfilingCommand() {
        addSequential(new EnableReverseTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.TWO_D_PROFILE_DRIVE_PERCENT, Drive.TWO_D_ACCEL_PERCENT, "RightHatchToCargoShip"));

        addSequential(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));

        addSequential(new EnableMotionProfile(new Distance(24, Distance.Unit.INCH), Distance.ZERO, Drive.ONE_D_PROFILE_DRIVE_PERCENT, Drive.ONE_D_PROFILE_ACCEL_PERCENT));

        addSequential(new LineSensorStrafeCommandGroup(0));
    
        addSequential(new ElevatorPositionCommand(Elevator.HATCH_PLACE_LOW_HEIGHT_ELEVATOR));

        addSequential(new ScoreHatchCommand());
    }

}
