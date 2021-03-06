package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep6Final implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(230,216,0,0));
        sWaypoints.add(new Waypoint(245,254,15,80,"PreparePlaceCube"));
        sWaypoints.add(new Waypoint(286,246,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(230, 216), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":230,"y":216},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":245,"y":254},"speed":80,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":286,"y":246},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromLeftStep6Final
}