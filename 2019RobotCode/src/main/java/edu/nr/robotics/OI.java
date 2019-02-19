package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.multicommands.ClimbCommand;
import edu.nr.robotics.multicommands.GetHatchStationCommand;
import edu.nr.robotics.multicommands.IntakeToggleCommand;
import edu.nr.robotics.multicommands.PrepareClimbCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveToBallCommand;
import edu.nr.robotics.subsystems.drive.DriveToSomethingJoystickCommand;
import edu.nr.robotics.subsystems.drive.DriveToTargetCommand;
import edu.nr.robotics.subsystems.drive.DumbDriveToggleCommand;
import edu.nr.robotics.subsystems.drive.EnableSniperForwardMode;
import edu.nr.robotics.subsystems.drive.EnableSniperTurnMode;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToAngleCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorSwitchToClimbGearCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorSwitchToElevatorGearCommand;
import edu.nr.robotics.subsystems.hatchmechanism.DeployHatchToggleCommand;
import edu.nr.robotics.subsystems.hatchmechanism.GrabHatchToggleCommand;
import edu.nr.robotics.subsystems.hatchmechanism.ScoreHatchCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersDeployToggleCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersScoreCommand;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollersToggleCommand;
import edu.nr.robotics.subsystems.sensors.ToggleLimelightCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI implements SmartDashboardSource {

    public static final double JOYSTICK_DEAD_ZONE = 0.2;

    public static final double SPEED_MULTIPLIER = 1.0;

    private static final int CANCEL_ALL_BUTTON_NUMBER = 0; //find all of these and make them too
    private static final int KID_MODE_SWITCH = 1;
    private static final int CARGO_TOP_BUTTON_NUMBER = 2;
    private static final int CARGO_MIDDLE_BUTTON_NUMBER = 3;
    private static final int CARGO_BOTTOM_BUTTON_NUMBER = 4;
    private static final int CARGO_PICKUP_BUTTON_NUMBER = 34;
    private static final int HATCH_TOP_BUTTON_NUMBER = 5;
    private static final int HATCH_MIDDLE_BUTTON_NUMBER = 6;
    private static final int HATCH_BOTTOM_BUTTON_NUMBER = 7;
    private static final int ELEVATOR_BOTTOM_BUTTON_NUMBER = 8;
    private static final int GROUND_PICKUP_HATCH_BUTTON_NUMBER = 9;
    private static final int CLIMB_HEIGHT_HIGH_BUTTON_NUMBER = 10;
    private static final int CLIMB_HEIGHT_LOW_BUTTON_NUMBER = 11;
    private static final int CLIMB_BUTTON_NUMBER = 12;
    private static final int HATCH_DEPLOY_TOGGLE_NUMBER = 13;
    private static final int HATCH_GRAB_TOGGLE_NUMBER = 14;
    private static final int HATCH_SCORE_NUMBER = 34;
    private static final int TOGGLE_INTAKE_NUMBER = 36;
    private static final int INTAKE_ROLLERS_DEPLOY_TOGGLE_NUMBER = 37;
    private static final int SWITCH_ELEVATOR_GEAR_NUMBER = 16;
    //private static final int INTAKE_ROLLERS_RUN_NUMBER = 17;
    private static final int INTAKE_ROLLERS_PUKE_NUMBER = 18;
    private static final int GET_HATCH_STATION_NUMBER = 19;
    //private static final int GET_HATCH_GROUND_NUMBER = 20;
    //private static final int GET_CARGO_NUMBER = 21;

    private static final int DRIVE_TO_CARGO_AUTO_NUMBER = 13;
    private static final int DRIVE_TO_CARGO_HYBRID_NUMBER = 4;
    private static final int DRIVE_TO_TARGET_AUTO_NUMBER = 7;
    private static final int DRIVE_TO_TARGET_HYBRID_NUMBER = 2;
    private static final int TURN_90_LEFT_NUMBER = 3;
    private static final int TURN_90_RIGHT_NUMBER = 4;
    private static final int TURN_180_NUMBER = 2;
    private static final int TOGGLE_LIMELIGHT_NUMBER = 8;
    private static final int RESET_GYRO_NUMBER = 10;
    private static final int SNIPER_MODE_FORWARD = 1;
    private static final int SNIPER_MODE_TURN = 1;
    private static final int DUMB_DRIVE_NUMBER = 14;

    private double driveSpeedMultiplier = 1;

    private static OI singleton;

    private final Joystick driveLeft;
    private final Joystick driveRight;

    private final Joystick operatorLeft;
    private final Joystick operatorRight;

    private final Joystick elevatorStick;
    private final Joystick liftStick;

    private JoystickButton kidModeSwitch;
    private JoystickButton elevGearSwitcherSwitch;

    private static final int STICK_LEFT = 0; //find these
    private static final int STICK_RIGHT = 1; 
    private static final int STICK_OPERATOR_LEFT = 2;
    private static final int STICK_OPERATOR_RIGHT = 3;

    public static final Drive.DriveMode driveMode = Drive.DriveMode.cheesyDrive; // set default type of drive here

    private OI() {
        driveLeft = new Joystick(STICK_LEFT);
        driveRight = new Joystick(STICK_RIGHT);

        operatorLeft = new Joystick(STICK_OPERATOR_LEFT);
        operatorRight = new Joystick(STICK_OPERATOR_RIGHT);

        elevatorStick = operatorRight;
        liftStick = operatorLeft;

       initDriveLeft();
       initDriveRight();

       initOperatorLeft();
       //initOperatorRight();

        SmartDashboardSource.sources.add(this);

    }

    public void initDriveLeft() {
        // hybrid track
        new JoystickButton(driveLeft, DRIVE_TO_CARGO_HYBRID_NUMBER).whenPressed(new DriveToSomethingJoystickCommand(Pipeline.Cargo));
        new JoystickButton(driveLeft, DRIVE_TO_CARGO_HYBRID_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));

        new JoystickButton(driveLeft, DRIVE_TO_TARGET_HYBRID_NUMBER).whenPressed(new DriveToSomethingJoystickCommand(Pipeline.Target));
        new JoystickButton(driveLeft, DRIVE_TO_TARGET_HYBRID_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));
                //auto track
        new JoystickButton(driveLeft, DRIVE_TO_CARGO_AUTO_NUMBER).whenPressed(new DriveToBallCommand());
        new JoystickButton(driveLeft, DRIVE_TO_CARGO_AUTO_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));

        new JoystickButton(driveLeft, DRIVE_TO_TARGET_AUTO_NUMBER).whenPressed(new DriveToTargetCommand());
        new JoystickButton(driveLeft, DRIVE_TO_TARGET_AUTO_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));

        //sniper mode
        new JoystickButton(driveLeft, SNIPER_MODE_FORWARD).whenPressed(new EnableSniperForwardMode(true));
        new JoystickButton(driveLeft, SNIPER_MODE_FORWARD).whenPressed(new EnableSniperForwardMode(false));

        //tuning command too

        //dumb drive
		new JoystickButton(driveLeft, DUMB_DRIVE_NUMBER).whenPressed(new DumbDriveToggleCommand());

    }

    public void initDriveRight() {
        new JoystickButton(driveRight, TOGGLE_LIMELIGHT_NUMBER).whenPressed(new ToggleLimelightCommand());

        new JoystickButton(driveRight, RESET_GYRO_NUMBER).whenPressed(new ResetGyroCommand());

       // new JoystickButton(driveRight, DUMB_DRIVE_NUMBER).whenPressed(new );

        new JoystickButton(driveRight, TURN_180_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), 
            new Angle(180, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
        new JoystickButton(driveRight, TURN_90_RIGHT_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), 
            new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
        new JoystickButton(driveRight, TURN_90_LEFT_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), 
            new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));

        //sniper mode
        new JoystickButton(driveLeft, SNIPER_MODE_TURN).whenPressed(new EnableSniperTurnMode(true));
        new JoystickButton(driveLeft, SNIPER_MODE_TURN).whenPressed(new EnableSniperTurnMode(false));
    }

    public void initOperatorLeft() {
       // new JoystickButton(operatorLeft, CANCEL_ALL_BUTTON_NUMBER).whenPressed(new CancelAllCommand());

        //kid mode boi
        kidModeSwitch = new JoystickButton(operatorLeft, KID_MODE_SWITCH);
        /*
        //elev gear boi
        elevGearSwitcherSwitch = new JoystickButton(operatorLeft, SWITCH_ELEVATOR_GEAR_NUMBER);
        elevGearSwitcherSwitch.whenPressed(new ElevatorSwitchToClimbGearCommand());
        elevGearSwitcherSwitch.whenReleased(new ElevatorSwitchToElevatorGearCommand());

        //cargo heights
        new JoystickButton(operatorLeft, CARGO_TOP_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.CARGO_PLACE_TOP_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, CARGO_MIDDLE_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.CARGO_PLACE_MIDDLE_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, CARGO_BOTTOM_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.CARGO_PLACE_LOW_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, CARGO_PICKUP_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.CARGO_PICKUP_HEIGHT_ELEVATOR));

        //hatch heights
        new JoystickButton(operatorLeft, HATCH_TOP_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.HATCH_PLACE_TOP_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, HATCH_MIDDLE_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.HATCH_PLACE_MIDDLE_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, HATCH_BOTTOM_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.HATCH_PLACE_LOW_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, GROUND_PICKUP_HATCH_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.HATCH_PICKUP_GROUND_HEIGHT_ELEVATOR));

        //climb heights
        new JoystickButton(operatorLeft, CLIMB_HEIGHT_LOW_BUTTON_NUMBER).whenPressed(new PrepareClimbCommand(Elevator.CLIMB_LOW_HEIGHT_ELEVATOR));
        new JoystickButton(operatorLeft, CLIMB_HEIGHT_HIGH_BUTTON_NUMBER).whenPressed(new PrepareClimbCommand(Elevator.CLIMB_HIGH_HEIGHT_ELEVATOR));


        //GEARSHIFT COMMAND GONE?
       // new JoyStickButton(operatorLeft, CLIMB_BUTTON_NUMBER).whenPressed(new Gear);
    
       //hatch
        new JoystickButton(operatorLeft, HATCH_DEPLOY_TOGGLE_NUMBER).whenPressed(new DeployHatchToggleCommand());
        new JoystickButton(operatorLeft, HATCH_GRAB_TOGGLE_NUMBER).whenPressed(new GrabHatchToggleCommand());
    
        new JoystickButton(operatorLeft, HATCH_SCORE_NUMBER).whenPressed(new ScoreHatchCommand());
        new JoystickButton(operatorLeft, GET_HATCH_STATION_NUMBER).whenPressed(new GetHatchStationCommand());

        //intake
        new JoystickButton(operatorLeft, TOGGLE_INTAKE_NUMBER).whenPressed(new IntakeRollersToggleCommand());
        new JoystickButton(operatorLeft, INTAKE_ROLLERS_PUKE_NUMBER).whenPressed(new IntakeRollersScoreCommand());
        new JoystickButton(operatorLeft, INTAKE_ROLLERS_DEPLOY_TOGGLE_NUMBER).whenPressed(new IntakeToggleCommand());

        //climb
        new JoystickButton(operatorLeft, CLIMB_BUTTON_NUMBER).whenPressed(new ClimbCommand());
*/
    }

    public void initOperatorRight() {
        
    }

    public static OI getInstance() {
        init();
        return singleton;
    }

    public static void init() {
        if(singleton == null) {
            singleton = new OI();
        }
    }

    public double getArcadeMoveValue() {
        return -snapDriveJoysticks(driveLeft.getY()) * Drive.MOVE_JOYSTICK_MULTIPLIER * SPEED_MULTIPLIER;
    }

    public double getArcadeTurnValue() {
        return -snapDriveJoysticks(driveRight.getX()) * Drive.TURN_JOYSTICK_MULTIPLIER * SPEED_MULTIPLIER;
    }


    public double getArcadeHValue() {
        return snapDriveJoysticks(driveLeft.getX()) * Drive.MOVE_JOYSTICK_MULTIPLIER * SPEED_MULTIPLIER;
    }

    public double getTankLeftValue() {
        return snapDriveJoysticks(driveLeft.getY());
    }

    public double getTankRightValue() {
        return snapDriveJoysticks(driveRight.getY());
    }

    public double getTankHValue() {
        return snapDriveJoysticks(driveLeft.getX());
    }

    public double getDriveLeftXValue() {
        return snapDriveJoysticks(driveLeft.getX());
    }

    public double getDriveRightXValue() {
        return snapDriveJoysticks(driveRight.getX());
    }

    public double getDriveLeftYValue() {
        return snapDriveJoysticks(driveLeft.getY());
    }

    public double getDriveRightYValue() {
        return snapDriveJoysticks(driveRight.getY());
    }

    public double getElevatorJoystickValue() {
		return snapDriveJoysticks(-elevatorStick.getX());
    }
    
    public double getLiftJoystickValue() {
        return snapDriveJoysticks(-liftStick.getX());
    }

    public double getDriveSpeedMultiplier(){
        return driveSpeedMultiplier;
    }

    private static double snapDriveJoysticks(double value) {
        if(Math.abs(value) < JOYSTICK_DEAD_ZONE) {
            value = 0;
        }else if (value > 0) {
            value -= JOYSTICK_DEAD_ZONE;
        } else {
            value += JOYSTICK_DEAD_ZONE;
        }
        value /= 1 - JOYSTICK_DEAD_ZONE;
        return value;
    }

    public double getRawMove() {
        return driveLeft.getY();
    }

    public double getRawTurn() {
        return driveRight.getX();
    }

    private double getTurnAdjust() {
        //do with buttons
        return 0;
    }

    public void smartDashboardInfo() {
        //add in sensor values for whatever we need
    }

    public boolean isTankNonZero() {
        return getTankLeftValue() != 0 || getTankRightValue() != 0 || getTankHValue() != 0;
    } 

    public boolean isArcadeNonZero() {
        return getArcadeMoveValue() != 0 || getArcadeTurnValue() != 0 || getArcadeHValue() != 0;
    }

    public boolean isDriveNonZero() {
        return getDriveLeftXValue() != 0 || getDriveRightXValue() != 0 || getDriveLeftYValue() != 0
        || getDriveRightYValue() != 0;
    }

    public boolean isElevatorNonZero() {
        return getElevatorJoystickValue() != 0;
    }

    public boolean isLiftNonZero() {
        return getLiftJoystickValue() != 0;
    }
    
    public boolean isHDriveZero() {
        return snapDriveJoysticks(driveLeft.getX()) == 0;
    }

    public boolean isKidModeOn(){
        //do later if needed
        return kidModeSwitch.get();
    }
}