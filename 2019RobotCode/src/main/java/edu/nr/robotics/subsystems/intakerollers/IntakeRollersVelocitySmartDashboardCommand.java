package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersVelocitySmartDashboardCommand extends NRCommand {

	boolean finished = false;
	double time = 0;
	double spiketime = 0;

	public IntakeRollersVelocitySmartDashboardCommand() {
		super(IntakeRollers.getInstance());
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().Vel_Setpoint);
		System.out.println("percent: " + IntakeRollers.getInstance().Vel_Setpoint);
	}

	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		finished = false;

		if (IntakeRollers.getInstance().getCurrent() > 40) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.15) {
				finished = true;
			} else {
				time = Timer.getFPGATimestamp();
			}
		}

		return finished;//(new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue());
	}
}