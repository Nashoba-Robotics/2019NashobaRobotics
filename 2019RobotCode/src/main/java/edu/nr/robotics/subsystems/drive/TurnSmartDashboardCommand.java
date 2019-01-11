package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.units.Angle;


public class TurnSmartDashboardCommand extends NRCommand {

    private DoublePIDOutput out;
    private Angle initialAngle;
    private GyroCorrection gyro;
    private boolean reachedSetVel = false;

    public TurnSmartDashboardCommand() {
        super(Drive.getInstance());
        this.out = Drive.getInstance();

    }

    public void onStart() {
        gyro = new GyroCorrection(Drive.angleToTurn, Drive.drivePercent);
        out.pidWrite(0,0);
        initialAngle = gyro.getAngleError().sub(Drive.angleToTurn);
        reachedSetVel = false;

    }

    public void onExecute() {

        double headingAdjustment = new gyro.getTurnValue(true);
        if(Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
            headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
        }

        double outputLeft, outputRight;


        if((Drive.getInstance().getLeftVelocity().abs.div(Drive.MAX_SPEED_DRIVE)) > Math.abs(headingAdjustment)
         || (Drive.getInstance().getRightVelocity().abs.div(Drive.MAX_SPEED_DRIVE)) > Math.abs(headingAdjustment)) {
            reachedSetVel = true;
        }

        if(!reachedSetVel) {
            outputLeft = -1*Math.signum(headingAdjustment);
            outputRight = -1*Math.signum(headingAdjustment);
        }else{
            outputLeft = -headingAdjustment;
            outputRight = headingAdjustment;
        }

        out.pidWrite(outputLeft, outputRight);

    }


    public void onEnd() {
        Drive.getInstance().disable();
    }

    public boolean isFinishedNR() {
        
        boolean finished = Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_TURN_SPEED_THRESHOLD) && 
                (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
                return finished;
    }



}