package edu.nr.robotics.auton.automap;


import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.LeftCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftHatchToRocketFrontProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftRocketBackToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftRocketBackToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToRocketBackProfilingCommand;

public class StartPosLeftRocketBackCommand extends CommandGroup {

    public StartPosLeftRocketBackCommand() {
        addSequential(new StartPosLeftToRocketBackProfilingCommand());

        addSequential(new ConditionalCommand(new LeftRocketBackToHatchProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new LeftRocketBackToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new LeftHatchToRocketFrontProfilingCommand()){

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
