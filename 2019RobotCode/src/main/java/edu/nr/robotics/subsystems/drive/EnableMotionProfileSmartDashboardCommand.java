package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableMotionProfileSmartDashboardCommand extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	Distance tempHPosition = Distance.ZERO;

	public EnableMotionProfileSmartDashboardCommand() {
		super(Drive.getInstance());
	}

	public void onStart() {
		Drive.getInstance().enableMotionProfiler(Drive.endX, Drive.endY, Drive.oneDDrivePercent, Drive.oneDAccelPercent);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
		initialHPosition = Drive.getInstance().getHPosition();
	}

	public void onExecute() {

		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {

			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putNumberArray("Motion Profiler V Left", new double [] {Drive.getInstance().pidGetLeft() , OneDimensionalMotionProfilerTwoMotor.velocityGoal });
			SmartDashboard.putNumberArray("Motion Profiler V Right", new double [] {Drive.getInstance().pidGetRight() , OneDimensionalMotionProfilerTwoMotor.velocityGoal});
			
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);

			SmartDashboard.putNumberArray("Motion Profiler X Left", new double [] {
					new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
					, new Distance(
									OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH) , 
					new Distance(OneDimensionalMotionProfilerTwoMotor.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH)});

			SmartDashboard.putNumberArray("Motion Profiler X Right", new double [] {
					new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH),
					new Distance(
								OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight,
								Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH),
					new Distance(OneDimensionalMotionProfilerTwoMotor.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
								.get(Distance.Unit.INCH)});
		}
	}

	public void onEnd() {
		Drive.getInstance().disableProfiler();
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
	}

	public boolean isFinishedNR() {

		boolean finished;

		System.out.println("thing1: " + new Distance((Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
		- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
				.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1))), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH));

		System.out.println("thing2: " + new Distance(Math.abs(Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
		- initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
				.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH));

		finished = Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
						.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)) < Drive.END_THRESHOLD
				.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) && 
				Math.abs(Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
						.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)) < Drive.END_THRESHOLD
				.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
								&& Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);
		return finished;
	}
}