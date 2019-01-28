package edu.nr.robotics.auton.automap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.LeftCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftHatchToRocketBackProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftRocketFrontToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftRocketFrontToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToRocketFrontProfilingCommand;
public class StartPosLeftRocketFrontCommand extends CommandGroup {

    public StartPosLeftRocketFrontCommand() {
        addSequential(new StartPosLeftToRocketFrontProfilingCommand());

        addSequential(new ConditionalCommand(new LeftRocketFrontToHatchProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new LeftRocketFrontToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new LeftHatchToRocketBackProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.rocket;
            }

        });

        addSequential(new ConditionalCommand(new LeftHatchToCargoShipProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
            }

        });

        addSequential(new ConditionalCommand(new LeftCargoToCargoShipProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
            }

        });
    }

}
