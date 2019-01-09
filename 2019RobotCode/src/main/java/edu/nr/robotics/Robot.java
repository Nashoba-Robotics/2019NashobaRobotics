package edu.nr.robotics;

import edu.nr.lib.network.LimelightNetworkTable;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

private double prevTime = 0;

private static Robot singleton;

public synchronized static Robot getInstance(){
    return singleton;
}

public void robotInit(){

    singleton = this;

    smartDashboardInit();
    autoChooserInit();
    OI.init();
    CameraInit();

    LimelightNetworkTable.getInstance().lightLED(false);

}
    
    public void autoChooserInit() {

    }

    public void smartDashboardInit() {

        SmartDashboard.putData(new CSVSaverEnable());
        SmartDashboard.putData(new CSVSaverDisable());
        SmartDashboard.putNumber("Auto Wait Time", 0);

        if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
                SmartDashboard.putData(new DriveForwardBasicSmartDashboardCommand());
                SmartDashboard.putData(new EnableMotionProfileSmartDashboardCommand());
			    SmartDashboard.putData(new DriveForwardSmartDashboardCommandH());
			    SmartDashboard.putData(new TurnSmartDashboardCommand());
        }

        if(EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new ElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new ElevatorMoveBasicSmartDashboardCommand());	
			SmartDashboard.putData(new ElevatorProfileSmartDashboardCommand());
        }

        @Override
        public void disabledInit() {
            for(NRSubsystem subsystem : NRSubsystem.subsystems) {
                subsystem.disable();
            }
        }
        @Override
        public void testInit(){

        }

        public void disabledPeriodic() {

        }

        public void autonomousInit() {

        }

        public void autonomousPeriodic() {

        }

        public void teleopInit() {
            //new CancelAllCommand().start(); maybe? depending on gameplay

           // LimelightNetworkTable.getInstance().lightLED(true);
           // LimelightNetworkTable.getInstance().lightLED(false);
        }

        public void teleopPeriodic() {

            double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
            prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        }

        public void CameraInit() {
            new Thread(() -> {
                UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
                camera.setResolution(720, 1080);
                
            }).start();
            
        }

        public void testPeriodic() {

        } 

        public void robotPeriodic() {

            Scheduler.getInstance().run();
            Periodic.runAll();
            SmartDashboardSource.runAll();

        }

        

    }

}