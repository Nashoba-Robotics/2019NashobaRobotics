package edu.nr.robotics.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
    //redo
    private static Drive singleton;

		private TalonSRX  leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, hDrive, hDriveFollow, pigeonTalon; 
		//these may change because of new talons




    public static Drive getInstance() {
		if (singleton == null)
			init();
		return singleton;
    }
    
    public synchronized static void init() {
		if (singleton == null) {
			singleton = new Drive();
			//singleton.setJoystickCommand(new DriveJoystickCommand());
		}
    }
		
		


}