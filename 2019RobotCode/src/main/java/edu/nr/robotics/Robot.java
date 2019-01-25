package edu.nr.robotics;

import edu.nr.lib.network.LimelightNetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.nr.robotics.subsystems.drive.EnableMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardSmartDashboardCommandH;
import edu.nr.robotics.subsystems.drive.TurnSmartDashboardCommand;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Destination;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.AutoChoosers.Platform;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
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

    private Command autonomusCommand;
    public AutoChoosers.StartPos selectedStartPos;
    public AutoChoosers.GamePiece selectedGamePiece;
    public AutoChoosers.GamePiece selectedGamePiece2;
    public AutoChoosers.Destination selectedDestination;
    public AutoChoosers.Platform selectedPlatform;
    public AutoChoosers.Destination2 selectedDestination2;
    public double autoWaitTime;
    
    public void autoChooserInit() {
        AutoChoosers.autoStartPosChooser.addDefault("Start Pos Left", StartPos.left);
        AutoChoosers.autoStartPosChooser.addObject("Start Pos Middle", StartPos.middle);
		AutoChoosers.autoStartPosChooser.addObject("Start Pos Right", StartPos.right);

        AutoChoosers.autoGamePiece1Chooser.addDefault("Game Piece", GamePiece.hatch);
        AutoChoosers.autoGamePiece1Chooser.addObject("Game Piece", GamePiece.cargo);

        AutoChoosers.autoGamePiece2Chooser.addDefault("Game Piece", GamePiece.hatch);
        AutoChoosers.autoGamePiece2Chooser.addObject("Game Piece", GamePiece.cargo);

        AutoChoosers.autoPlatformChooser.addDefault("Platform", Platform.no);
        AutoChoosers.autoPlatformChooser.addObject("Platform", Platform.yes);

        AutoChoosers.autoDestination1Chooser.addDefault("Destination", Destination.rocketFront);
        AutoChoosers.autoDestination1Chooser.addObject("Destination", Destination.rocketBack);
        AutoChoosers.autoDestination1Chooser.addObject("Destination", Destination.cargoShipFrontLeft);
        AutoChoosers.autoDestination1Chooser.addObject("Destination", Destination.cargoShipFrontRight);
        AutoChoosers.autoDestination1Chooser.addObject("Destination", Destination.cargoShipSide);

        AutoChoosers.autoDestination2Chooser.addDefault("Destination 2", Destination2.rocket);
        AutoChoosers.autoDestination2Chooser.addObject("Destination 2", Destination2.cargoShip);


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
                SmartDashboard.putData(new EnableTwoDMotionProfileSmartDashboardCommand());
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

