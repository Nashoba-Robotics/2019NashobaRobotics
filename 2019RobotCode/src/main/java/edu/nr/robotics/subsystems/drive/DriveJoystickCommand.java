package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;

public class DriveJoystickCommand extends JoystickCommand {

    double prevTime = 0;

    private GyroCorrection gyroCorrection;

    public DriveJoystickCommand() {
        super(Drive.getInstance());
    }

    public void onStart() {
        gyroCorrection = new GyroCorrection();
    }

    public void onExecute() {
        double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
        prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        switch (OI.driveMode) {
            case arcadeDrive:
                double moveValue = OI.getInstance().getArcadeMoveValue();
                double rotateValue = OI.getInstance().getArcadeTurnValue();
                double hValue = OI.getInstance().getArcadeHValue();

                moveValue = NRMath.powWithSign(moveValue, 3);
                rotateValue = NRMath.powWithSign(rotateValue, 5);
                hValue = NRMath.powWithSign(hValue, 3);

                if(Math.abs(rotateValue) < 0.05 && (Math.abs(moveValue)) > 0.05 || Math.abs(hValue) > 0.05){
                    rotateValue -= gyroCorrection.getTurnValue(Drive.kP_thetaOneD, false);
                } else{
                    gyroCorrection.clearInitialValue();
                }


                Drive.getInstance().arcadeDrive(moveValue, rotateValue, hValue);

                break;

                case tankDrive:
                    double left = OI.getInstance().getTankLeftValue();
                    double right = OI.getInstance().getTankRightValue();
                    double hDrive = OI.getInstance().getTankHValue();

                    left = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(left);
                    right = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(right);

                    right = NRMath.powWithSign(right, 3);
                    left = NRMath.powWithSign(left, 3);
                    hDrive = NRMath.powWithSign(hDrive, 3);

                    break;

                case cheesyDrive:
                    double cheesyMoveValue = OI.getInstance().getArcadeMoveValue();
                    double cheesyRotateValue = OI.getInstance().getArcadeTurnValue();
                    double cheesyHValue = OI.getInstance().getArcadeHValue();    
                    
                    cheesyMoveValue = NRMath.powWithSign(cheesyMoveValue, 2);
                    cheesyRotateValue = NRMath.powWithSign(cheesyRotateValue, 2);
                    cheesyHValue = NRMath.powWithSign(cheesyHValue, 2);
                    
                    Drive.getInstance().cheesyDrive(cheesyMoveValue, cheesyRotateValue, cheesyHValue);

                    break;

                case fieldCentricDrive:
                    Angle robotAngle = Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).getYaw().sub(new Angle(90, Angle.Unit.DEGREE));
                    
                    double inputForward = OI.getInstance().getArcadeMoveValue();
                    double inputSide = OI.getInstance().getArcadeHValue();
                    double fieldCentricRotate = OI.getInstance().getArcadeTurnValue();
                    
                    double inputMoveMagnitude = Math.sqrt(Math.pow(inputForward, 2) + Math.pow(inputSide, 2));
                    Angle inputAngle = new Angle(Math.atan2(inputForward, inputSide),Angle.Unit.RADIAN);
                
                    double fieldCentricMoveValue = -inputMoveMagnitude * Math.cos(inputAngle.sub(robotAngle).get(Angle.Unit.RADIAN));
                    double fieldCentricHValue = inputMoveMagnitude * Math.cos(inputAngle.sub(robotAngle).sub(new Angle(90,Angle.Unit.DEGREE)).get(Angle.Unit.RADIAN));

                    Drive.getInstance().cheesyDrive(fieldCentricMoveValue, fieldCentricRotate, fieldCentricHValue);
    
                break;

              }      
    }

    public boolean shouldSwitchToJoystick() {
        if (!(Drive.getInstance().getCurrentCommand() instanceof DriveToSomethingJoystickCommand) /*&& !Robot.getInstance().isAutonomous()*/) {

            if((OI.driveMode == Drive.DriveMode.arcadeDrive) || (OI.driveMode == Drive.DriveMode.cheesyDrive || OI.driveMode == Drive.DriveMode.fieldCentricDrive)) {
                return OI.getInstance().isArcadeNonZero();
            } else {
                return OI.getInstance().getTankLeftValue() != 0 || OI.getInstance().getTankRightValue() != 0;
            }

        }

        return false;

    }

    public long getPeriodOfCheckingForSwitchToJoystick() {
        return 100;
    }

}

