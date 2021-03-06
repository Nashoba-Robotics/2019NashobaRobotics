package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftRightFromRightStep3 implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(220,84,0,0));
        sWaypoints.add(new Waypoint(225,66,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(220, 84), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA: [{"position":{"x":220,"y":84},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":225,"y":66},"speed":20,"radius":0,"marker":"","comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: RightRightFromRightStep4
}