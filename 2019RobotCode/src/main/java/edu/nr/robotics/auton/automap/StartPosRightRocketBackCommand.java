package edu.nr.robotics.auton.automap;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.autoroutes.RightCargoToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightHatchToCargoShipProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightHatchToRocketFrontProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightRocketBackToCargoProfilingCommand;
import edu.nr.robotics.auton.autoroutes.RightRocketBackToHatchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToRocketBackProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosRightRocketBackCommand extends CommandGroup{

    public StartPosRightRocketBackCommand() {
        addSequential(new StartPosRightToRocketBackProfilingCommand());

        addSequential(new ConditionalCommand(new RightRocketBackToHatchProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new RightRocketBackToCargoProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

        addSequential(new ConditionalCommand(new RightHatchToRocketFrontProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.rocket && Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new RightHatchToCargoShipProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.rocket && Robot.getInstance().selectedGamePiece2 == GamePiece.hatch;
            }

        });

        addSequential(new ConditionalCommand(new RightCargoToCargoShipProfilingCommand()){

            protected boolean condition() {
                return Robot.getInstance().selectedDestination2 == Destination2.cargoShip && Robot.getInstance().selectedGamePiece2 == GamePiece.cargo;
            }

        });

    }

}
