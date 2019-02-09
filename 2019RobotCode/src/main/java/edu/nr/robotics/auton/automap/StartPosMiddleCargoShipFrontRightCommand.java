package edu.nr.robotics.auton.automap;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.CargoShipFrontRightToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipFrontRightToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToCargoShipFrontLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToCargoShipFrontRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosMiddleCargoShipFrontRightCommand extends CommandGroup{

    public StartPosMiddleCargoShipFrontRightCommand() {

        addSequential(new StartPosMiddleToCargoShipFrontRightProfilingCommand());

        addSequential(new ConditionalCommand(new CargoShipFrontRightToHatchProfilingCommand()){
            
            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new CargoShipFrontRightToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
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
