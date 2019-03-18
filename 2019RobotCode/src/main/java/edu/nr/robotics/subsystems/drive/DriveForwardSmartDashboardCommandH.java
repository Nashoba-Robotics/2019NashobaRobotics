package edu.nr.robotics.subsystems.drive;


import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;

public class DriveForwardSmartDashboardCommandH extends NRCommand {

    Distance initialPosition;
    GyroCorrection gyro;
    double percent;

    public DriveForwardSmartDashboardCommandH() {
        super(Drive.getInstance());
        gyro = new GyroCorrection();
    }

    public void onStart() {
        Drive.getInstance().setMotorSpeedInPercent(0, 0, Drive.getInstance().oneDDrivePercent);
        initialPosition = Drive.getInstance().getHPosition();
        gyro.reset();
    }

    public void onEnd() {
        Drive.getInstance().setMotorSpeedInPercent(0,0,0);
    }

    protected boolean isFinishedNR() {
        return (Drive.getInstance().getHPosition().sub(initialPosition)).abs().greaterThan(Drive.endY.abs());
    }

}
