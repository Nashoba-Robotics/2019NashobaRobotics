/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.nr.robotics;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;

import edu.nr.lib.motorcontrollers.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	TalonSRX talon;
	/*TalonSRX carriageTalon;
	TalonSRX intakeRoller1;
	TalonSRX intakeRoller2;*/
	//CANSparkMax spark1;
	//CANSparkMax spark2;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		talon = CTRECreator.createMasterTalon(12);
		//carriageTalon = CTRECreator.createFollowerTalon(1, talon);
		//intakeRoller1 = CTRECreator.createFollowerTalon(6, talon);
		//intakeRoller2 = CTRECreator.createMasterTalon(4);

		talon.configContinuousCurrentLimit(40, 0);
		talon.configPeakCurrentLimit(50, 0);
		talon.configPeakCurrentDuration(1000, 0);

		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		/*carriageTalon.configContinuousCurrentLimit(40, 0);
		carriageTalon.configPeakCurrentLimit(50, 0);
		carriageTalon.configPeakCurrentDuration(1000, 0);

		intakeRoller1.configContinuousCurrentLimit(40, 0);
		intakeRoller1.configPeakCurrentLimit(50, 0);
		intakeRoller1.configPeakCurrentDuration(1000, 0);*/

		//intakeRoller2.configContinuousCurrentLimit(40, 0);
		///intakeRoller2.configPeakCurrentLimit(50, 0);
		//intakeRoller2.configPeakCurrentDuration(1000, 0);

		SmartDashboard.putNumber("Motor Percent: ", 0);
		//SmartDashboard.putNumber("Spark2 Percent: ", 0);

		//spark1 = SparkMax.createSpark(RobotMap.SPARK1_ID, true);
		//spark2 = SparkMax.createSpark(RobotMap.SPARK2_ID, true);
		//spark1.setIdleMode(IdleMode.kCoast);
		//spark2.setIdleMode(IdleMode.kCoast);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		talon.set(ControlMode.PercentOutput, 0);
//		carriageTalon.set(ControlMode.PercentOutput, 0);
//		intakeRoller1.set(ControlMode.PercentOutput, 0);
//		intakeRoller2.set(ControlMode.PercentOutput, 0);
//		spark1.disable();
//		spark2.disable();

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		talon.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Motor Percent: ", 0));
		System.out.println( SmartDashboard.getNumber("Motor Percent: ", 0));
		//carriageTalon.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Motor Percent: ", 0));
		//intakeRoller1.set(ControlMode.PercentOutput, -SmartDashboard.getNumber("Motor Percent 2: ", 0));
		//intakeRoller2.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Motor Percent: ", 0));
	
		//SmartDashboard.putNumber("Talon 1 Current: ", talon.getOutputCurrent());
		//SmartDashboard.putNumber("Talon 2 Current: ", carriageTalon.getOutputCurrent());
		//SmartDashboard.putNumber("Speed", talon.getSelectedSensorVelocity(0)/1365.33);

//		spark1.set(SmartDashboard.getNumber("Spark1 Percent: ", 0));
//		spark2.set(SmartDashboard.getNumber("Spark2 Percent: ", 0));
//		
//		SmartDashboard.putNumber("Spark1 Velocity: ", spark1.getEncoder().getVelocity());
//		SmartDashboard.putNumber("Spark1 Position: ", spark1.getEncoder().getPosition());
//		SmartDashboard.putNumber("Spark1 Current: ", spark1.getOutputCurrent());
//
//		SmartDashboard.putNumber("Spark2 Velocity: ", spark2.getEncoder().getVelocity());
//		SmartDashboard.putNumber("Spark2 Position: ", spark2.getEncoder().getPosition());
//		SmartDashboard.putNumber("Spark2 Current: ", spark2.getOutputCurrent());
		SmartDashboard.putNumber("main Current: ", talon.getOutputCurrent());
		//SmartDashboard.putNumber("Follow1 Current: ", carriageTalon.getOutputCurrent());
		//SmartDashboard.putNumber("Follow2 current: ", intakeRoller1.getOutputCurrent());

		SmartDashboard.putNumber("Encoder", talon.getSelectedSensorPosition());

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
