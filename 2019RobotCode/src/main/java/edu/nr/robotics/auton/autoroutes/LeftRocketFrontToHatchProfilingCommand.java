

package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class LeftRocketFrontToHatchProfilingCommand extends CommandGroup{

    public LeftRocketFrontToHatchProfilingCommand() {
        addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.TWO_D_PROFILE_DRIVE_PERCENT, Drive.TWO_D_ACCEL_PERCENT, "LeftRocketFrontToHatch"));

        addSequential(new ConditionalCommand(new ElevatorPositionCommand(Elevator.getInstance().HATCH_PLACE_LOW_HEIGHT_ELEVATOR)){

            protected boolean condition() {
				return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
			}

        });

        //grab the hatch
    }

}
