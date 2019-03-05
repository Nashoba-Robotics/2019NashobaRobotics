package edu.nr.robotics.subsystems.intakerollers;


import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersIntakeCommand extends NRCommand {

	boolean finished = false;
	double spiketime = 0;
	double time = 0;
	
	public IntakeRollersIntakeCommand() {
		super(IntakeRollers.getInstance());
	}
	
	@Override
	public void onStart() {
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().INTAKE_PERCENT);
	}

	
	@Override
	public void onEnd() {
		IntakeRollers.getInstance().disable();
		
	}
	
	@Override
	public boolean isFinishedNR() {
		finished = false;

		if (IntakeRollers.getInstance().getCurrent() > 40) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.15) {
				finished = true;
			} else {
				time = Timer.getFPGATimestamp();
			}
		}

		return finished;//(!EnabledSensors.cargoIntakeSensorOne.get() && !EnabledSensors.cargoIntakeSensorTwo.get() && !EnabledSensors.cargoIntakeSensorThree.get()) || isSensorBroken; /*|| IntakeRollers.getInstance().getCurrentLeft() > IntakeRollers.PEAK_CURRENT_INTAKE_ROLLERS || IntakeRollers.getInstance().getCurrentRight() > IntakeRollers.PEAK_CURRENT_INTAKE_ROLLERS*/
	}
}