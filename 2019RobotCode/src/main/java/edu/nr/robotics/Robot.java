package edu.nr.robotics;

import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Destination;
import edu.nr.robotics.auton.AutoChoosers.Destination2;
import edu.nr.robotics.auton.AutoChoosers.GamePiece;
import edu.nr.robotics.auton.AutoChoosers.Platform;
import edu.nr.robotics.auton.AutoChoosers.SandstormType;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.automap.StartPosLeftCargoShipSideCommand;
import edu.nr.robotics.auton.automap.StartPosLeftRocketBackCommand;
import edu.nr.robotics.auton.automap.StartPosLeftRocketFrontCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleCargoShipFrontLeftCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleCargoShipFrontRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightCargoShipSideCommand;
import edu.nr.robotics.auton.automap.StartPosRightRocketBackCommand;
import edu.nr.robotics.auton.automap.StartPosRightRocketFrontCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.auxiliarydrive.AuxiliaryDrive;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardSmartDashboardCommandH;
import edu.nr.robotics.subsystems.drive.EnableMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.EnableReverseTwoDMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.TurnSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorSwitchGearCommand;
import edu.nr.robotics.subsystems.hatchmechanism.DeployHatchToggleCommand;
import edu.nr.robotics.subsystems.hatchmechanism.GrabHatchToggleCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployToggleCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersIntakeCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersVelocitySmartDashboardCommand;
import edu.nr.robotics.subsystems.lift.Lift;
import edu.nr.robotics.subsystems.lift.LiftMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.lift.LiftSetPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismDeployCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismRetractCommand;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanismToggleCommand;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    private static Robot singleton;

    private static double period = 0.01;

    private double prevTime = 0;

    private Command autonomousCommand;
    public AutoChoosers.SandstormType selectedSandstormType;
    public AutoChoosers.StartPos selectedStartPos;
    public AutoChoosers.GamePiece selectedGamePiece;
    public AutoChoosers.GamePiece selectedGamePiece2;
    public AutoChoosers.Destination selectedDestination;
    public AutoChoosers.Platform selectedPlatform;
    public AutoChoosers.Destination2 selectedDestination2;
    public double autoWaitTime;
    public Compressor robotCompressor;

    public synchronized static Robot getInstance() {
        return singleton;
    }

    public void robotInit() {
        singleton = this;

        m_period = period; // period that code runs at

        smartDashboardInit();
        autoChooserInit();
        OI.init();
        Drive.getInstance();
        Elevator.getInstance();
        IntakeRollers.getInstance();
        Lift.getInstance();
        AuxiliaryDrive.getInstance();

        robotCompressor = new Compressor(RobotMap.PCM_ID);
        robotCompressor.start();

        // CameraInit();

        LimelightNetworkTable.getInstance().lightLED(false);
        //System.out.println("end of robot init");

    }

    public void autoChooserInit() {
        AutoChoosers.sandstormTypeChooser.addDefault("Auto", SandstormType.auto);
        AutoChoosers.sandstormTypeChooser.addObject("Driver", SandstormType.driver);

        AutoChoosers.autoStartPosChooser.addDefault("Left", StartPos.left);
        AutoChoosers.autoStartPosChooser.addObject("Middle", StartPos.middle);
        AutoChoosers.autoStartPosChooser.addObject("Right", StartPos.right);

        AutoChoosers.autoGamePiece1Chooser.addDefault("Hatch", GamePiece.hatch);
        AutoChoosers.autoGamePiece1Chooser.addObject("Cargo", GamePiece.cargo);

        AutoChoosers.autoGamePiece2Chooser.addDefault("Hatch", GamePiece.hatch);
        AutoChoosers.autoGamePiece2Chooser.addObject("Cargo", GamePiece.cargo);

        AutoChoosers.autoPlatformChooser.addDefault("No", Platform.no);
        AutoChoosers.autoPlatformChooser.addObject("Yes", Platform.yes);

        AutoChoosers.autoDestination1Chooser.addDefault("Rocket Front", Destination.rocketFront);
        AutoChoosers.autoDestination1Chooser.addObject("Rocket Back", Destination.rocketBack);
        AutoChoosers.autoDestination1Chooser.addObject("Cargo Ship Front Left", Destination.cargoShipFrontLeft);
        AutoChoosers.autoDestination1Chooser.addObject("Cargo Ship Front Right", Destination.cargoShipFrontRight);
        AutoChoosers.autoDestination1Chooser.addObject("Cargo Ship Side", Destination.cargoShipSide);

        AutoChoosers.autoDestination2Chooser.addDefault("Rocket", Destination2.rocket);
        AutoChoosers.autoDestination2Chooser.addObject("Cargo Ship", Destination2.cargoShip);

        SmartDashboard.putData("Sandstorm Type", AutoChoosers.sandstormTypeChooser);
        SmartDashboard.putData("Start Pos", AutoChoosers.autoStartPosChooser);
        //SmartDashboard.putData("Starting Game Piece", AutoChoosers.autoGamePiece1Chooser);
        SmartDashboard.putData("2nd Game Piece", AutoChoosers.autoGamePiece2Chooser);
        SmartDashboard.putData("Platform Start", AutoChoosers.autoPlatformChooser);
        SmartDashboard.putData("1st Destination", AutoChoosers.autoDestination1Chooser);
        SmartDashboard.putData("2nd Destination", AutoChoosers.autoDestination2Chooser);

    }

    public void smartDashboardInit() {

        SmartDashboard.putData(new CSVSaverEnable());
        SmartDashboard.putData(new CSVSaverDisable());
        SmartDashboard.putNumber("Auto Wait Time", 0);

        if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new DriveForwardBasicSmartDashboardCommand());
            SmartDashboard.putData(new EnableMotionProfileSmartDashboardCommand());
			SmartDashboard.putData(new DriveForwardSmartDashboardCommandH());
			SmartDashboard.putData(new TurnSmartDashboardCommand());
            SmartDashboard.putData(new EnableTwoDMotionProfileSmartDashboardCommand());
            SmartDashboard.putData(new EnableReverseTwoDMotionProfileSmartDashboardCommand());
        }

        if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new ElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new ElevatorMoveBasicSmartDashboardCommand());	
            SmartDashboard.putData(new ElevatorProfileSmartDashboardCommandGroup());
            SmartDashboard.putData(new ElevatorPositionSmartDashboardCommand());
            SmartDashboard.putData(new ElevatorSwitchGearCommand());
        }

        if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new IntakeRollersVelocitySmartDashboardCommand());
            SmartDashboard.putData(new IntakeRollersDeployToggleCommand());
            SmartDashboard.putData(new IntakeRollersDeployCommand());
            SmartDashboard.putData(new IntakeRollersIntakeCommand());
        }

        if (EnabledSubsystems.LIFT_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new LiftSetPositionSmartDashboardCommand());
            SmartDashboard.putData(new LiftMoveBasicSmartDashboardCommand());
        }

        if (EnabledSubsystems.HATCH_MECHANISM_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new GrabHatchToggleCommand());
            SmartDashboard.putData(new DeployHatchToggleCommand());
        }

        if (EnabledSubsystems.LIFT_LOCK_MECHANISM_SMARTDASHBOARD_DEBUG_ENABLED) {
            SmartDashboard.putData(new LiftLockMechanismToggleCommand());
            SmartDashboard.putData(new LiftLockMechanismDeployCommand());
            SmartDashboard.putData(new LiftLockMechanismRetractCommand());
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
            enabledInit();
        }

        public void disabledPeriodic() {

        }

        public void autonomousInit() {
            enabledInit();

            selectedSandstormType = AutoChoosers.sandstormTypeChooser.getSelected();
            selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
            selectedDestination = AutoChoosers.autoDestination1Chooser.getSelected();
            selectedDestination2 = AutoChoosers.autoDestination2Chooser.getSelected();
            selectedGamePiece = AutoChoosers.autoGamePiece1Chooser.getSelected();
            selectedGamePiece2 = AutoChoosers.autoGamePiece2Chooser.getSelected();
            selectedPlatform = AutoChoosers.autoPlatformChooser.getSelected();

            autonomousCommand = new DoNothingCommand(); //getAutoCommand();

            if(autonomousCommand != null)
                autonomousCommand.start();

        }

        public void autonomousPeriodic() {

        }

        public void teleopInit() {
            enabledInit();
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

        public void enabledInit() {
            new LiftLockMechanismRetractCommand().start();
        }
    
        public Command getAutoCommand() {
            if (selectedSandstormType == SandstormType.driver) {
                return new DoNothingCommand();
            } else if(selectedStartPos == StartPos.left && selectedDestination == Destination.rocketBack) {
                return new StartPosLeftRocketBackCommand();
            } else if(selectedStartPos == StartPos.left && selectedDestination == Destination.rocketFront) {
                return new StartPosLeftRocketFrontCommand();
            } else if(selectedStartPos == StartPos.left && selectedDestination == Destination.cargoShipSide) {
                return new StartPosLeftCargoShipSideCommand();
            } else if(selectedStartPos == StartPos.middle && selectedDestination == Destination.cargoShipFrontLeft) {
                return new StartPosMiddleCargoShipFrontLeftCommand();
            } else if(selectedStartPos == StartPos.middle && selectedDestination == Destination.cargoShipFrontRight) {
                return new StartPosMiddleCargoShipFrontRightCommand();
            } else  if(selectedStartPos == StartPos.right && selectedDestination == Destination.rocketBack) {
                return new StartPosRightRocketBackCommand();
            } else if(selectedStartPos == StartPos.right && selectedDestination == Destination.rocketFront) {
                return new StartPosRightRocketFrontCommand();
            } else if(selectedStartPos == StartPos.right && selectedDestination == Destination.cargoShipSide) {
                return new StartPosRightCargoShipSideCommand();
            }
            return new DriveOverBaselineAutoCommand();
        }

        public double getPeriod() {
            return period;
        }

    }

