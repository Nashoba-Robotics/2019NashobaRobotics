package edu.nr.robotics.auton.automap;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.CargoShipFrontLeftToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipFrontLeftToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.LeftHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToCargoShipFrontLeftProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosMiddleCargoShipFrontLeftCommand extends CommandGroup{

    public StartPosMiddleCargoShipFrontLeftCommand() {

        addSequential(new StartPosMiddleToCargoShipFrontLeftProfilingCommand());

        addSequential(new ConditionalCommand(new CargoShipFrontLeftToHatchProfilingCommand()){
            
            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new CargoShipFrontLeftToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
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
