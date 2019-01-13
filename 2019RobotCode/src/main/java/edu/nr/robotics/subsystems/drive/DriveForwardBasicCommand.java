package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;


public class DriveForwardBasicCommand extends NRCommand {

    double percent;
    Distance distance;
    Distance initialPosition;
    GyroCorrection gyro;

    public DriveForwardBasicCommand(Distance distance) {
            this(distance, Drive.PROFILE_DRIVE_PERCENT);
    }

    public DriveForwardBasicCommand(Distance distance, double percent) {
        super(Drive.getInstance());
        this.percent = percent;
        this.distance = distance;
        gyro = new GyroCorrection();
    }

    public void onStart() {
        initialPosition = Drive.getInstance().getLeftPosition();
        gyro.reset();
    }

    public void onExecute() {
        Drive.getInstance().setMotorSpeedInPercent(0,0,0);
    }

    public boolean isFinishedNR() {
        return (Drive.getInstance().getLeftPosition().sub(initialPosition)).abs().greaterThan(distance.abs());
    }



}