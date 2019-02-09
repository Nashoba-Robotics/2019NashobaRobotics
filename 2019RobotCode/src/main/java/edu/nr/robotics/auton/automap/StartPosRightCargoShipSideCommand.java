package edu.nr.robotics.auton.automap;

import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.CargoLeftToCargoShipSlot2ProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoRightToCargoShipSlot2ProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipRightSideToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipRightSideToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.HatchLeftToRocketFrontProfilingCommand;
import edu.nr.robotics.auton.autoroutes.HatchRightToCargoShipSlot2ProfilingCommand;
import edu.nr.robotics.auton.autoroutes.HatchRightToRocketFrontProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToShipSideProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosRightCargoShipSideCommand extends CommandGroup{

    public StartPosRightCargoShipSideCommand() {
        addSequential(new StartPosRightToShipSideProfilingCommand());

        addSequential(new ConditionalCommand(new CargoShipRightSideToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new CargoShipRightSideToHatchProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new HatchRightToRocketFrontProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.rocket;
            }


        });

        addSequential(new ConditionalCommand(new HatchRightToCargoShipSlot2ProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
            }

        });

        addSequential(new ConditionalCommand(new CargoRightToCargoShipSlot2ProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo && Robot.getInstance().selectedDestination2 == Destination2.cargoShip;
            }


        });


    }

}
