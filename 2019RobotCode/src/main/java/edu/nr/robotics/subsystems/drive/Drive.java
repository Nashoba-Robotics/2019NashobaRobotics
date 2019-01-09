package edu.nr.robotics.subsystems.drive;

public class Drive {
    //redo
    private static Drive singleton;

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