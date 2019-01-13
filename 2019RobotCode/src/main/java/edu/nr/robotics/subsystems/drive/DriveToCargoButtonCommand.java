package edu.nr.robotics.subsystems.drive;



import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveToCargoButtonCommand extends CommandGroup {



	public DriveToCargoButtonCommand(boolean advanced) {

			

		addSequential(new ConditionalCommand(new DriveToCargoCommandAdvanced(), new DriveToCargoCommandBasic()) {
	
			protected boolean condition() {
				return advanced;
			}
		});
	}
}