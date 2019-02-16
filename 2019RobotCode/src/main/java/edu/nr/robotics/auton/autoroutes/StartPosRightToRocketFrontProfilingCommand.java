package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.AutoChoosers.Platform;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersReverseCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosRightToRocketFrontProfilingCommand extends CommandGroup{
    public StartPosRightToRocketFrontProfilingCommand(){
        addSequential(new ConditionalCommand(
                new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO,
                        Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT, "StartPosRightToRocketFront"),
                new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO,
                        Angle.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT,
                        "StartPosRightPlatformToRocketFront")) {

            @Override
            protected boolean condition() {
                return Robot.getInstance().selectedPlatform == Platform.no;
            }

        });
    
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
