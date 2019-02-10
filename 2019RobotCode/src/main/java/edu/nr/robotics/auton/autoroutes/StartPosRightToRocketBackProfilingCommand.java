package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersReverseCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosRightToRocketBackProfilingCommand extends CommandGroup{
    public StartPosRightToRocketBackProfilingCommand(){
        addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "StartPosRightToRocketBack"));
    
        addSequential(new ConditionalCommand(new ElevatorPositionCommand(Elevator.getInstance().CARGO_PLACE_LOW_HEIGHT_ELEVATOR), new ElevatorPositionCommand(Elevator.getInstance().HATCH_PLACE_LOW_HEIGHT_ELEVATOR)){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new IntakeRollersReverseCommand(IntakeRollers.getInstance().OUTTAKE_PERCENT)){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });
    
    }
}
