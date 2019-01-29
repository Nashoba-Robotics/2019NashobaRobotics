package edu.nr.robotics.auton.automap;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.RightCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightHatchToRocketBackProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightRocketFrontToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightRocketFrontToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToRocketFrontProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosRightRocketFrontCommand extends CommandGroup{

public StartPosRightRocketFrontCommand() {
    addSequential(new StartPosRightToRocketFrontProfilingCommand());

    addSequential(new ConditionalCommand(new RightRocketFrontToHatchProfilingCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
        }

    });

    addSequential(new ConditionalCommand(new RightRocketFrontToCargoProfilingCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
        }


    });

    addSequential(new ConditionalCommand(new RightHatchToRocketBackProfilingCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.rocket;
        }

    });

    addSequential(new ConditionalCommand(new RightHatchToCargoShipProfilingCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
        }

    });

    addSequential(new ConditionalCommand(new RightCargoToCargoShipProfilingCommand()){

        protected boolean condition() {
            return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
        }

    });



}

}