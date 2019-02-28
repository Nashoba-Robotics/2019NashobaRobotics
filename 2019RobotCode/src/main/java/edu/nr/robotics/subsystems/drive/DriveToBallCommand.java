package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.Equation;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.SensorVoting;
import edu.wpi.first.wpilibj.Timer;

public class DriveToBallCommand extends NRCommand {

    double time = 0;
	double spiketime = 0;

    //returns the distance in inches based on the height of the box drawn by the limelight
    Equation dist = new Equation() {
    
        @Override
        public double getValue(double x) {
            return 2280.64 / (10.6409 + 0.831893*x);
        }
    };

    //returns the ideal vel percent based on the distance in inches
    Equation velPercent = new Equation() {

        @Override
        public double getValue(double x) {
            return Math.pow(x, 1.6) / 1000;
        }

    };

    double boxHeight;
    double targetDistance;
    double moveValue;
    double headingAdjustment;
    double outputLeft;
    double outputRight;

	public DriveToBallCommand() {
		super(Drive.getInstance());
    }
	
	@Override
	protected void onStart() {
        //new EnableLimelightCommand(true).start();
        LimelightNetworkTable.getInstance().setPipeline(Pipeline.Cargo);
	}
	
	@Override
	protected void onExecute() {
        
        boxHeight = LimelightNetworkTable.getInstance().getBoxHeight();
        
        if (boxHeight != 0) {
            targetDistance = dist.getValue(boxHeight);
            moveValue = velPercent.getValue(targetDistance);
         } else {
            targetDistance = 0;
            moveValue = 0;
        }

        if (moveValue > 1) {
            moveValue = 1;
        }

        moveValue *= 0.7;
 
		
        headingAdjustment =  -0.25 * Math.sin(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN));
        /*((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) 
				/ ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) 
				* (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) 
				* -LimelightNetworkTable.getInstance().getHorizOffset().signum();*/
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
        
        System.out.println("move val: " + moveValue);
        System.out.println("heading adjustment: " + headingAdjustment);

		outputLeft = moveValue - headingAdjustment;
		outputRight = moveValue + headingAdjustment;
		
		Drive.getInstance().setMotorSpeedInPercent(outputLeft, outputRight, 0);
	}
	
	@Override
	protected void onEnd() {
        new EnableLimelightCommand(false).start();
        LimelightNetworkTable.getInstance().setPipeline(Pipeline.DriverCam);
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

		return finished;
		//return new SensorVoting(EnabledSensors.cargoIntakeSensorOne, EnabledSensors.cargoIntakeSensorTwo, EnabledSensors.cargoIntakeSensorThree).isTrue();
	}
}
