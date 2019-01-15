package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;

public class DriveToCargoCommandBasic extends NRCommand {
    private boolean hasStartedForward = false;

	private boolean finished = false;

	private GyroCorrection gyro;
	
	public DriveToCargoCommandBasic() {
		super(new NRSubsystem[] {Drive.getInstance()});
		gyro = new GyroCorrection();
    }
    
    protected void onStart() {
		hasStartedForward = false;
		new EnableLimelightCommand(true).start();
		gyro.reset();

		if ((Elevator.getInstance().getPosition().sub(Elevator.CARGO_PICKUP_HEIGHT_ELEVATOR)).abs().greaterThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR)) {
			finished = true;
		} else {
			finished = false;
		}
    }
    
    protected void onExecute() {

		double headingAdjustment = 0;
		double outputLeft = 0;
		double outputRight = 0;

		if (!hasStartedForward) {

			headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) / ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) * (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) * -LimelightNetworkTable.getInstance().getHorizOffset().signum();

			if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
				headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
            }			
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
		}
		
		if (LimelightNetworkTable.getInstance().getHorizOffset().abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD)

				&& !LimelightNetworkTable.getInstance().getHorizOffset().abs().equals(Angle.ZERO) 
				&& !hasStartedForward) {
			hasStartedForward = true;
			gyro.reset();
		}

		if (hasStartedForward = true) {
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
			
			outputLeft += Drive.DRIVE_TO_CARGO_PERCENT;
			outputRight += Drive.DRIVE_TO_CARGO_PERCENT;
		}
		Drive.getInstance().pidWrite(outputLeft, outputRight);
    }

    protected void onEnd() {
		new EnableLimelightCommand(false).start();;
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
    }
    
    protected boolean isFinishedNR() {
		return (!(new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue())) || finished;
	}

}