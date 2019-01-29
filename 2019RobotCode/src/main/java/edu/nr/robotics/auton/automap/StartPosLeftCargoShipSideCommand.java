package edu.nr.robotics.auton.automap;

import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.CargoLeftToCargoShipSlot2ProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipLeftSideToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.CargoShipLeftSideToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.HatchLeftToCargoShipSlot2ProfilingCommand;
import edu.nr.robotics.auton.autoroutes.HatchLeftToRocketFrontProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToCargoShipSideProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosLeftCargoShipSideCommand extends CommandGroup {

    public StartPosLeftCargoShipSideCommand() {
        addSequential(new StartPosLeftToCargoShipSideProfilingCommand());

        addSequential(new ConditionalCommand(new CargoShipLeftSideToHatchProfilingCommand()){

            @Override
            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }
            
        });

        addSequential(new ConditionalCommand(new CargoShipLeftSideToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new HatchLeftToRocketFrontProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.rocket && Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new HatchLeftToCargoShipSlot2ProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.cargoShip && Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new CargoLeftToCargoShipSlot2ProfilingCommand()){
            
            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.cargoShip && Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });


    }

}