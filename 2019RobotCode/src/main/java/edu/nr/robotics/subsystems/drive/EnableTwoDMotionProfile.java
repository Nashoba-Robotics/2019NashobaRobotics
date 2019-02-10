package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableTwoDMotionProfile extends NRCommand {

		Distance initialLeftPosition;
		Distance initialRightPosition;
		Distance tempLeftPosition = Distance.ZERO;
		Distance tempRightPosition = Distance.ZERO;
		
		Distance endX;
		Distance endY;
		Angle endAngle;
		Distance xPoint1;
		Distance yPoint1;
		Angle anglePoint1;
		double drivePercent;
		double accelPercent;
		String profileName;

		private double profileStartTime = 0;
		private double profileStartTimeMs = 0;
		private int index = 0;

		private final Distance END_THRESHOLD = Drive.END_THRESHOLD;

		public EnableTwoDMotionProfile(Distance endX, Distance endY, Angle endAngle, Distance xPoint1, Distance yPoint1, Angle anglePoint1Angle, double drivePercent, double accelPercent, String profileName) {
			super(Drive.getInstance());
			this.endX = endX;
			this.endY = endY;
			this.endAngle = endAngle;
			this.xPoint1 = xPoint1;
			this.yPoint1 = yPoint1;
			this.anglePoint1 = anglePoint1;
			this.drivePercent = drivePercent;
			this.accelPercent = accelPercent;
			this.profileName = profileName;
		}

		@Override
		public void onStart() {
			Drive.getInstance().enableTwoDMotionProfiler(endX, endY, endAngle, xPoint1, yPoint1, anglePoint1, drivePercent,
					accelPercent, profileName);
			initialLeftPosition = Drive.getInstance().getLeftPosition();
			initialRightPosition = Drive.getInstance().getRightPosition();
			profileStartTime = 0;
			profileStartTimeMs = 0;
		}

		@Override
		public void onExecute() {
			if (TwoDimensionalMotionProfilerPathfinder.twoDEnabled && profileStartTime == 0) {
				profileStartTime = Timer.getFPGATimestamp();
				profileStartTimeMs = (profileStartTime * 1000);
			} else if (TwoDimensionalMotionProfilerPathfinder.twoDEnabled) {
				index = (int) Math.round(((Timer.getFPGATimestamp() * 1000) - profileStartTimeMs) / 20.0);

				if (index < TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory().length()) {
					Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
					SmartDashboard.putNumberArray("Motion Profiler V Left", new double[] {
							new Speed(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
									Time.Unit.HUNDRED_MILLISECOND).get(Distance.Unit.FOOT, Time.Unit.SECOND),
							new Speed(
									TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory().get(index).velocity,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND)
											.get(Distance.Unit.FOOT, Time.Unit.SECOND) });
					SmartDashboard.putNumberArray("Motion Profiler V Right",
							new double[] {
									new Speed(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
											Time.Unit.HUNDRED_MILLISECOND).get(Distance.Unit.FOOT, Time.Unit.SECOND),
									new Speed(
											TwoDimensionalMotionProfilerPathfinder.modifier.getRightTrajectory()
													.get(index).velocity,
											Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND)
													.get(Distance.Unit.FOOT, Time.Unit.SECOND) });

					Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);

					SmartDashboard
							.putNumberArray("Motion Profiler X Left",
									new double[] {
											new Distance(
													(Drive.getInstance().pidGetLeft() - initialLeftPosition
															.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)),
													Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.FOOT),
											new Distance(TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory()
													.get(index).position, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
															.get(Distance.Unit.FOOT) });
					SmartDashboard
							.putNumberArray("Motion Profiler X Right",
									new double[] {
											new Distance(
													Drive.getInstance().pidGetRight() - initialRightPosition
															.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE),
													Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.FOOT),
											new Distance(TwoDimensionalMotionProfilerPathfinder.modifier
													.getRightTrajectory().get(index).position,
													Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.FOOT) });

				}
			}

		}

		@Override
		public void onEnd() {
			Drive.getInstance().disableProfiler();
			Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
		}

		@Override
		public boolean isFinishedNR() {

			boolean finished;

			finished = (Drive.getInstance().getLeftPosition().sub(initialLeftPosition)).sub(new Distance(
				TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory()
					.get(TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory().length() - 1).position,
				Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)).abs().lessThan(Drive.getInstance().END_THRESHOLD)
			
				&& (Drive.getInstance().getRightPosition().sub(initialRightPosition)).sub(new Distance(
				TwoDimensionalMotionProfilerPathfinder.modifier.getRightTrajectory()
					.get(TwoDimensionalMotionProfilerPathfinder.modifier.getRightTrajectory().length() - 1).position,
				Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)).abs().lessThan(Drive.getInstance().END_THRESHOLD)
			
				&& Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);


			return finished;
			
		}

}
