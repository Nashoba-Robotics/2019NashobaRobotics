package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class CargoShipFrontRightToCargoProfilingCommand extends CommandGroup {

    public CargoShipFrontRightToCargoProfilingCommand() {
        addSequential(new EnableTwoDMotionProfile(Distance.ZERO, Distance.ZERO, Angle.ZERO, Distance.ZERO, Distance.ZERO, Angle.ZERO, Drive.TWO_D_PROFILE_DRIVE_PERCENT, Drive.TWO_D_ACCEL_PERCENT, "CargoShipFrontRightToCargo"));
    
        addSequential(new ConditionalCommand(new ElevatorPositionCommand(Elevator.getInstance().REST_HEIGHT_ELEVATOR)){

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
