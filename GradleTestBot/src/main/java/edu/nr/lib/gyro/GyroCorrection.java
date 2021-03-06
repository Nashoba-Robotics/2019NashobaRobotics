package edu.nr.lib.gyro;

import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.drive.Drive;

public class GyroCorrection
{
	public static final double DEFAULT_KP_THETA = 0.05;
	public double MAX_ANGLE_CORRECTION_SPEED = 0.2;
	protected boolean initialized = false;
	
	private Angle initialAngle;
	Angle goalAngle;
	Gyro gyro;
	
	public GyroCorrection(Angle angle) {
			this.gyro = Pigeon.getPigeon(Drive.getInstance().getPigeonTalon());
		goalAngle = angle;
		initialAngle = gyro.getYaw();
	}
	
	public GyroCorrection() {
		this(Angle.ZERO);
	}
	
	public GyroCorrection(Angle angle, double MAX_ANGLE_CORRECTION_SPEED, Drive drive) {
		this(angle);
		this.MAX_ANGLE_CORRECTION_SPEED = MAX_ANGLE_CORRECTION_SPEED;
		
	}

	/**
	 * Get the speed that the robot should turn at, capped at {@value #MAX_ANGLE_CORRECTION_SPEED}. It is based linearly off of kP_theta. 
	 * @param kP_theta The factor the angle should be changed by.
	 * @return A speed, on a scale from -1 to 1, from -{@value #MAX_ANGLE_CORRECTION_SPEED} to {@value #MAX_ANGLE_CORRECTION_SPEED}.
	 */
	public double getTurnValue(double kP_theta)
	{
		if(initialized == false)
		{
			reset();
			initialized = true;
		}
		
		double turn = -getAngleError().get(Angle.Unit.DEGREE) * kP_theta;
    	if(turn < 0)
    		turn = Math.max(-MAX_ANGLE_CORRECTION_SPEED, turn);
    	else
    		turn = Math.min(MAX_ANGLE_CORRECTION_SPEED, turn);
    	
    	return turn;
	}
	
	/**
	 * Calls {@link #getTurnValue(double)} with the default value of {@value #DEFAULT_KP_THETA}.
	 * @return
	 */
	public double getTurnValue()
	{
		return this.getTurnValue(DEFAULT_KP_THETA);
	}
	
	/**
	 * Get the angle error
	 */
	public Angle getAngleError()
	{
		if(initialized == false)
		{
			reset();
			initialized = true;
		}
		Angle currentAngle = gyro.getYaw();
				
		//Error is just based off initial angle
    	return currentAngle.sub(initialAngle).add(goalAngle);
	}
	
	/**
	 * Sets the current angle offset to zero, 
	 * so if {@link GyroCorrection#getAngleErrorDegrees} were called immediately afterward, it would return zero.
	 */
	public void reset()
	{
		initialAngle = gyro.getYaw();
	}		
	/**
	 * Causes the initial angle value to be reset the next time getTurnValue() is called. Use this in the end() and interrupted()
	 * functions of commands to make sure when the commands are restarted, the initial angle value is reset.
	 */
	public void clearInitialValue()
	{
		initialized = false;
	}
}
