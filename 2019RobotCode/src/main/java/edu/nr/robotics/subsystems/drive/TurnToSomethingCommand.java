package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.Timer;

public class TurnToSomethingCommand extends NRCommand {

    private Pipeline pipeline;

    private boolean reachedSetVel = false;
    private double initTime;

    public TurnToSomethingCommand(Pipeline pipeline) {
        super(Drive.getInstance());
        this.pipeline = pipeline;
    }

    protected void onStart() {
        if(pipeline == Pipeline.Target)
            new EnableLimelightCommand(true).start();

        LimelightNetworkTable.getInstance().setPipeline(pipeline);

        reachedSetVel = false;
        initTime = Timer.getFPGATimestamp();
    }

    protected void onExecute() {
        double headingAdjustment;

        headingAdjustment = (headingAdjustment = -0.25 * Math.sin(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN)));
                
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
        LimelightNetworkTable.getInstance().setPipeline(Pipeline.DriverCam);
        new EnableLimelightCommand(false).start();
    }

    protected boolean isFinishedNR(){
        boolean finished = Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD)
        && Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD) && 
        LimelightNetworkTable.getInstance().getHorizOffset().lessThan(Drive.DRIVE_ANGLE_THRESHOLD) &&
        ((Timer.getFPGATimestamp() - initTime) > 0.5);
    
        return finished;
    }

}
