package org.usfirst.frc.team195.robot.Autonomous.Modes;

import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.SamplePath;
import org.usfirst.frc.team195.robot.Autonomous.Paths.SecondPath;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StraightPath;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class BasicMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new SamplePath();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new DrivePathAction(pathContainer));

		runAction(new WaitAction(2));

		runAction(new DrivePathAction(new SecondPath()));

		runAction(new WaitAction(15));
	}
}
