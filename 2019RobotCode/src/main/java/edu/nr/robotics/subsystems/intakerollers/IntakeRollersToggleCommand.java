package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersToggleCommand extends NRCommand {

    public IntakeRollersToggleCommand() {
        super(IntakeRollers.getInstance());
    }

    protected void onStart() {

        if(IntakeRollers.getInstance().isRunning()) {
            IntakeRollers.getInstance().disable();
        } else if (IntakeRollers.getInstance().isIntakeRollersDeployed()) {
            new IntakeRollersIntakeCommand().start();
            //IntakeRollers.getInstance().setMotorPercent(IntakeRollers.INTAKE_PERCENT);
        }
    }

    protected boolean isFinishedNR() {
        return true;
    }

}