/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeExecuter;
import org.usfirst.frc.team195.robot.Autonomous.Modes.BasicMode;
import org.usfirst.frc.team195.robot.Autonomous.Modes.NewMode;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.Loops.RobotStateEstimator;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;

public class Robot extends RobbieRobot {
	private Controllers robotControllers;

	private Looper mLooper;


	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;

	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private AutoModeExecuter autoModeExecuter;

	private LEDController ledController;

	private KnightJoystick driveJoystickThrottle;


	@Override
	public void robotInit() {
		robotControllers = Controllers.getInstance();
		mLooper = new Looper();

		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		driveBaseSubsystem.init();
		driveBaseSubsystem.registerEnabledLoops(mLooper);
		robotStateEstimator = RobotStateEstimator.getInstance();
		mLooper.register(robotStateEstimator);


		ledController = LEDController.getInstance();
		ledController.start();
		ledController.setRequestedState(LEDController.LEDState.FIXED_ON);

	}

	@Override
	public void autonomous() {
		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();


		AutoModeBase autoMode = new NewMode();


		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;

		autoModeExecuter.start();

		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
	}

	@Override
	public void operatorControl() {
		exitAuto();
		mLooper.start(false);

		while (isOperatorControl() && isEnabled()) {
			double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
			double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

			driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));
			threadRateControl.doRateControl(20);
		}
	}

	@Override
	public void disabled() {
		exitAuto();

		mLooper.stop();

		threadRateControl.start(true);

		while (isDisabled()) {
			driveBaseSubsystem.setBrakeMode(false);
			threadRateControl.doRateControl(100);
		}
	}

	private void exitAuto() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			autoModeExecuter = null;
		} catch (Throwable t) {

		}
	}

	@Override
	public void test() {
	}
}
