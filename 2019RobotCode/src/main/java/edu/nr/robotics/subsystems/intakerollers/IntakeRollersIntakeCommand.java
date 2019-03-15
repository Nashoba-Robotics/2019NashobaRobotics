package edu.nr.robotics.subsystems.intakerollers;


import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersIntakeCommand extends NRCommand {

	public IntakeRollersIntakeCommand() {
		super(IntakeRollers.getInstance());
	}
	
	@Override
	public void onStart() {
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().INTAKE_PERCENT);
	}
	
	public void onEnd() {
		new IntakeRollersWaitForBallCommand().start();
	}
	
}