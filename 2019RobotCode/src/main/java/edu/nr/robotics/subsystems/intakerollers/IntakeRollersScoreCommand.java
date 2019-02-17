package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeRollersScoreCommand extends CommandGroup {

    public IntakeRollersScoreCommand() {

        addSequential(new IntakeRollersReverseCommand(IntakeRollers.OUTTAKE_PERCENT));
        addSequential(new WaitCommand(IntakeRollers.SCORE_TIME.get(Time.Unit.SECOND)));
        addSequential(new IntakeRollersVelocityCommand(0));
    }

}