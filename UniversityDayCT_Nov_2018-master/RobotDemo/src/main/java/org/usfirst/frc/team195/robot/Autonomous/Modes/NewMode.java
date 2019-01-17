package org.usfirst.frc.team195.robot.Autonomous.Modes;

import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.FlashLEDsAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.MarkerPath;
import org.usfirst.frc.team195.robot.Autonomous.Paths.SamplePath;
import org.usfirst.frc.team195.robot.Autonomous.Paths.SecondPath;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StraightPath;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class NewMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new MarkerPath();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("OurMarker"),
						new FlashLEDsAction())))));
	}
}
