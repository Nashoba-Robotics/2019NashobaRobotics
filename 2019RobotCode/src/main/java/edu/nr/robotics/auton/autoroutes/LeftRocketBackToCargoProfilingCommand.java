
package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class LeftRocketBackToCargoProfilingCommand extends CommandGroup {

    public LeftRocketBackToCargoProfilingCommand() {
       addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "LeftRocketBackToCargo"));
    
       addSequential(new ConditionalCommand(new ElevatorPositionCommand(Elevator.getInstance().CARGO_PICKUP_HEIGHT_ELEVATOR)){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
        }

       });

       addSequential(new ConditionalCommand(new IntakeRollersIntakeCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
        }

       });
    }

}
