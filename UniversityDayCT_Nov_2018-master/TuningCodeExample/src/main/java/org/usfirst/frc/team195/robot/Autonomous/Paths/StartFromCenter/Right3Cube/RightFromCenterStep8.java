package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right3Cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenterStep8 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(99,147,0,0));
		sWaypoints.add(new Waypoint(49,139,0,80));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(99, 147), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}