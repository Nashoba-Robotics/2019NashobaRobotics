package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersVelocitySmartDashboardCommand extends NRCommand {
	
	double time = 0;
	double spiketime = 0;

	public IntakeRollersVelocitySmartDashboardCommand() {
		super(IntakeRollers.getInstance());
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().setVel);
		System.out.println("percent: " + IntakeRollers.getInstance().setVel);
	}

	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = false;

		if (IntakeRollers.getInstance().getCurrent() > 40) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.15) 
				finished = true;
		}
		else {
			time = Timer.getFPGATimestamp();
		}

		return finished;//(new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue());
	}
}