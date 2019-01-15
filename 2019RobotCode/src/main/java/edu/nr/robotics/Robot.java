package edu.nr.robotics;

import edu.nr.lib.network.LimelightNetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.nr.robotics.subsystems.drive.EnableMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardSmartDashboardCommandH;
import edu.nr.robotics.subsystems.drive.TurnSmartDashboardCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicSmartDashboardCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.commandbased.NRSubsystem;
//import com.revrobtoics.*;

public class Robot extends TimedRobot {

private double prevTime = 0;

private static Robot singleton;

public synchronized static Robot getInstance(){
    return singleton;
}

public void robotInit(){
    singleton = this;

    m_period = 0.01;  //period that code runs at

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
			SmartDashboard.putData(new ElevatorProfileSmartDashboardCommandGroup());
        }
    }

        @Override
        public void disabledInit() {
            for(NRSubsystem subsystem : NRSubsystem.subsystems) {
                subsystem.disable();
            }
        }
        @Override
        public void testInit() {

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
        @Override
        public void testPeriodic() {

        } 
        @Override
        public void robotPeriodic() {

            Scheduler.getInstance().run();
            Periodic.runAll();
            SmartDashboardSource.runAll();

        }
    
    

    }

