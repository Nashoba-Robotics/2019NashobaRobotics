package edu.nr.robotics.subsystems.intakerollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;

public class IntakeRollersVelocitySmartDashboardCommand extends NRCommand {

	public IntakeRollersVelocitySmartDashboardCommand() {
		super(IntakeRollers.getInstance());
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorPercent(IntakeRollers.getInstance().Vel_Setpoint);
	}

	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue());
	}
}