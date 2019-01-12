package edu.nr.robotics;
import edu.wpi.first.wpilibj.Joystick;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.elevator.Elevator;


public class OI implements SmartDashboardSource {

    public static final double JOYSTICK_DEAD_ZONE = 0.2;

    public static final double SPEED_MULTIPLIER = 1.0;

    private static final int CANCEL_ALL_BUTTON_NUMBER; //find all of these and make them too
    private static final int KID_MODE_SWITCH ;

    private double driveSpeedMultiplier = 1;

    private static OI singleton;

    private final Joystick driveLeft;
    private final Joystick driveRight;

    private final Joystick operatorLeft;
    private final Joystick operatorRight;

    private static final int STICK_LEFT; //find these
    private static final int STICK_RIGHT; 
    private static final int STICK_OPERATOR_LEFT;
    private static final int STICK_OPERATOR_RIGHT;

    public static final Drive.DriveMode driveMode = Drive.DriveMode.cheesyDrive; // set default type of drive here

    private OI() {
        driveLeft = new Joystick(STICK_LEFT);
        driveRight = new Joystick(STICK_RIGHT);

        operatorLeft = new Joystick(STICK_OPERATOR_LEFT);
        operatorRight = new Joystick(STICK_OPERATOR_RIGHT);

        initDriveLeft();
        initDriveRight();

        initOperatorLeft();
        initOperatorRight();

        SmartDashboardSource.sources.add(this);

    }

    public void initDriveLeft() {
        //make joystickbuttons later if necessary

        //tuning command too

    }

    public void initDriveRight() {
        //same as left but not
    }

    public void initOperatorLeft() {
        //do later
    }

    public void initOperatorRight() {
        //do later also
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
        return -snapDriveJoysticks(driveRight.getX()) * Drive.TURN_JOYSTICK_MULTIPLIER *SPEED_MULTIPLIER;
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
    
    public boolean isHDriveZero() {
        return snapDriveJoysticks(driveLeft.getX()) == 0;
    }

    public boolean isKidModeOn(){
        //do later if needed
    }
}