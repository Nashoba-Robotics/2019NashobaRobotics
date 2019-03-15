package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;

public class DriveToSomethingJoystickCommand extends NRCommand {

    Pipeline pipeline;

    public DriveToSomethingJoystickCommand(Pipeline pipeline) {
        super(Drive.getInstance());
        this.pipeline = pipeline;
    }

    protected void onStart() {
        if(pipeline == Pipeline.Target)
            new EnableLimelightCommand(true).start();
        LimelightNetworkTable.getInstance().setPipeline(pipeline);
    }

    //make sure limelight tracking cargo

    protected void onExecute() {
        double moveValue = NRMath.powWithSign(OI.getInstance().getArcadeMoveValue(), 2);

        double headingAdjustment;

        headingAdjustment = -0.25 * Math.sin(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN));/*((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) 
				/ ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) 
				* (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) 
				* -LimelightNetworkTable.getInstance().getHorizOffset().signum();*/

		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {

			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);

        }

		double outputLeft = moveValue - headingAdjustment;
        double outputRight = moveValue + headingAdjustment;   

        Drive.getInstance().setMotorSpeedInPercent(outputLeft, outputRight, 0);

    }

    protected void onEnd() {
        new EnableLimelightCommand(false).start();
        LimelightNetworkTable.getInstance().setPipeline(Pipeline.DriverCam);

    }

    protected boolean isFinishedNR() {
		return false;
	}

}