package edu.nr.robotics;
import edu.wpi.first.wpilibj.Joystick;
import edu.nr.lib.interfaces.SmartDashboardSource;


public class OI implements SmartDashboardSource {

    public static final double JOYSTICK_DEAD_ZONE = 0.2;

    public static final double SPEED_MULTIPLIER = 1.0;

    private static final int CANCEL_ALL_BUTTON_NUMBER; //find all of these and make them too
    private static final int KID_MODE_SWITCH = 2;

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

    public static final Drive.DriveMode drivemode = Drive.Drivemode.drive; // set default type of drive here

    private OI() {
        driveLeft = new Joystick(STICK_LEFT);
        driveRight = new Joystick(Stick_RIght);

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

}