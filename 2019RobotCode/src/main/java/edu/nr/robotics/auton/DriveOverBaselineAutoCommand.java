package edu.nr.robotics.auton;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveOverBaselineAutoCommand extends CommandGroup {
	
	public DriveOverBaselineAutoCommand() {

		addSequential(new DriveForwardBasicCommand(FieldMeasurements.WALL_TO_BASELINE));

	}
}
