package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;

public class TurnToCargoCommand extends NRCommand {

    private boolean reachedSetVel = false;

    public TurnToCargoCommand() {
        super(Drive.getInstance());
    }

    protected void onStart() {
        new EnableLimelightCommand(true).start();
        reachedSetVel = false;
    }

    protected void onExecute() {
        double headingAdjustment;

        headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) 
				/ ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) 
				* (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) 
                * -LimelightNetworkTable.getInstance().getHorizOffset().signum();
                
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		double outputLeft, outputRight;		

		if ((Drive.getInstance().getLeftVelocity().abs().div(Drive.MAX_SPEED_DRIVE)) > Math.abs(headingAdjustment) 
				|| (Drive.getInstance().getRightVelocity().abs().div(Drive.MAX_SPEED_DRIVE)) > Math.abs(headingAdjustment)) {
			reachedSetVel = true;
		}

		if (!reachedSetVel) {
			outputLeft = -1*Math.signum(headingAdjustment);
			outputRight = 1*Math.signum(headingAdjustment);
		} else {
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
		}
		
		Drive.getInstance().setMotorSpeedInPercent(outputLeft, outputRight, 0);
    }
    
    protected void onEnd() {
            new EnableLimelightCommand(false).start();
    }

    protected boolean isFinishedNR(){
        boolean finished = Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD)
        && Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD) && 
        LimelightNetworkTable.getInstance().getHorizOffset().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
    
        return finished;
    }

}
